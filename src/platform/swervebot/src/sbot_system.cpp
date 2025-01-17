/*
 * @file sbot_system.cpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "swervebot/sbot_system.hpp"

#include "stopwatch/stopwatch.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
SbotSystem::SbotSystem(const SbotConfig& config) : config_(config) {}

bool SbotSystem::Initialize() {
  // create a control context
  ControlContext context;
  context.config = config_;
  context.js_axis_queue = std::make_shared<ThreadSafeQueue<AxisEvent>>();
  context.js_button_queue = std::make_shared<ThreadSafeQueue<JsButton>>();
  context.rc_lever_queue = std::make_shared<ThreadSafeQueue<LeverEvent>>();
  context.command_queue = std::make_shared<ThreadSafeQueue<UserCommand>>();
  context.feedback_queue = std::make_shared<ThreadSafeQueue<RobotFeedback>>();

  // set up hardware
  if (config_.control_settings.user_input.type ==
      SbotConfig::UserInputType::kJoystick) {
    // initialize joystick
    joystick_ = std::make_unique<JoystickHandler>(
        config_.control_settings.user_input.joystick.device);
    if (!joystick_->Open()) {
      XLOG_ERROR("Failed to open joystick device");
      return false;
    }
    joystick_->RegisterJoystickButtonEventCallback(
        std::bind(&SbotSystem::OnJsButtonEvent, this, std::placeholders::_1,
                  std::placeholders::_2));
    joystick_->RegisterJoystickAxisEventCallback(
        std::bind(&SbotSystem::OnJsAxisEvent, this, std::placeholders::_1,
                  std::placeholders::_2));

    hid_event_listener_ = std::make_shared<HidEventListener>();
    if (!hid_event_listener_->AddHidHandler(joystick_.get())) {
      XLOG_ERROR("Failed to add joystick handler to event listener");
      return false;
    }
    hid_event_listener_->StartListening();
  } else if (config_.control_settings.user_input.type ==
             SbotConfig::UserInputType::kRcReceiver) {
    sbus_rc_ = std::make_shared<SbusReceiver>(
        config_.control_settings.user_input.rc_receiver.port);
    if (!sbus_rc_->Open()) {
      XLOG_ERROR("Failed to open SBUS serial port");
      return false;
    }
    sbus_rc_->SetOnRcMessageReceivedCallback(
        std::bind(&SbotSystem::OnSbusMsgReceived, this, std::placeholders::_1));
  } else {
    XLOG_ERROR("Unsupported control input type specified");
    return false;
  }

  // initialize robot base
  sbot_ = std::make_shared<WsSbotBase>(config_.base_settings);
  if (!sbot_->Initialize()) {
    XLOG_ERROR("Failed to initialize robot base");
    return false;
  }
  context.robot_base = sbot_;

  // initialize fsm
  AutoMode initial_state{context};
  fsm_ =
      std::make_unique<SbotFsm>(std::move(initial_state), std::move(context));

  return true;
}

void SbotSystem::ControlLoop() {
  Timer timer;
  bool first_run = true;
  keep_control_loop_ = true;
  auto& context = fsm_->GetContext();

  XLOG_INFO("SbotSystem: entering control loop");
  TimePoint last_time = Clock::now();
  while (keep_control_loop_) {
    timer.reset();

    // calculate time step
    if (first_run) {
      last_time = Clock::now();
      first_run = false;
      continue;
    }
    auto now = Clock::now();
    double dt =
        std::chrono::duration_cast<std::chrono::microseconds>(now - last_time)
            .count() /
        1000000.0f;
    last_time = now;

    // update control
    UserCommand cmd;
    while (context.command_queue->TryPop(cmd)) {
      sbot_->SetMotionCommand({{cmd.vx, cmd.vy, 0}, {0, 0, cmd.wz}});
    }
    sbot_->Update(dt);

    // time housekeeping
    if ((dt - 0.05) / 0.05 > 0.1) {
      XLOG_WARN("SbotSystem: control loop running at {} ms", dt * 1000);
    }

    timer.sleep_until_ms(50);
  }
  XLOG_INFO("SbotSystem: control loop exited");
}

void SbotSystem::OnJsButtonEvent(const JsButton& btn,
                                 const JxButtonEvent& event) {
  if (fsm_ == nullptr) return;

  //  std::cout << "Button " << (int)(btn) << " "
  //            << (event == JxButtonEvent::kPress ? "pressed" : "released")
  //            << std::endl;
  if (btn == JsButton::kStart && event == JxButtonEvent::kPress) {
    fsm_->GetContext().js_button_queue->Push(btn);
  }
}

void SbotSystem::OnJsAxisEvent(const JsAxis& axis, const float& value) {
  if (fsm_ == nullptr) return;

  //  std::cout << "Axis " << (int)(axis) << " value: " << value << std::endl;
  if (axis == JsAxis::kX || axis == JsAxis::kY || axis == JsAxis::kRX) {
    AxisEvent event;
    event.axis = axis;
    event.value = value;
    fsm_->GetContext().js_axis_queue->Push(event);
  }
}

void SbotSystem::OnSbusMsgReceived(const RcMessage& msg) {
  if (fsm_ == nullptr) return;

  // std::cout << "Sbus message received" << std::endl;
  // for (int i = 0; i < 18; ++i) {
  //   std::cout << "ch" << i << ": " << msg.channels[i] << " ";
  // }

  static RcMessage prev_msg;
  auto mode_chn = config_.control_settings.user_input.rc_receiver.mapping.mode;
  auto lx_chn =
      config_.control_settings.user_input.rc_receiver.mapping.linear_x;
  auto ly_chn =
      config_.control_settings.user_input.rc_receiver.mapping.linear_y;
  auto az_chn =
      config_.control_settings.user_input.rc_receiver.mapping.angular_z;

  if (msg.frame_loss) return;

  if (prev_msg.channels[mode_chn.channel] != msg.channels[mode_chn.channel]) {
    LeverEvent event;
    event.channel = mode_chn.channel;
    float level_pos = RcReceiverInterface::ScaleChannelValue(
        msg.channels[mode_chn.channel], mode_chn.min, mode_chn.neutral,
        mode_chn.max);
    if (level_pos > 0.5) level_pos = 1.0;
    if (level_pos < -0.5) level_pos = -1.0;
    if (mode_chn.invert) level_pos = -level_pos;
    event.value = level_pos;
    fsm_->GetContext().rc_lever_queue->Push(event);
  }

  // mimic joystick axis event
  if (prev_msg.channels[lx_chn.channel] != msg.channels[lx_chn.channel]) {
    AxisEvent event;
    float lx = RcReceiverInterface::ScaleChannelValue(
        msg.channels[lx_chn.channel], lx_chn.min, lx_chn.neutral, lx_chn.max);
    event.timestamp = Clock::now();
    event.axis = JsAxis::kX;
    if (lx_chn.invert) lx = -lx;
    event.value = lx;
    fsm_->GetContext().js_axis_queue->Push(event);
  }

  if (prev_msg.channels[ly_chn.channel] != msg.channels[ly_chn.channel]) {
    AxisEvent event;
    float ly = RcReceiverInterface::ScaleChannelValue(
        msg.channels[ly_chn.channel], ly_chn.min, ly_chn.neutral, ly_chn.max);
    event.timestamp = Clock::now();
    event.axis = JsAxis::kY;
    if (ly_chn.invert) ly = -ly;
    event.value = ly;
    fsm_->GetContext().js_axis_queue->Push(event);
  }

  if (prev_msg.channels[az_chn.channel] != msg.channels[az_chn.channel]) {
    AxisEvent event;
    float az = RcReceiverInterface::ScaleChannelValue(
        msg.channels[az_chn.channel], az_chn.min, az_chn.neutral, az_chn.max);
    event.timestamp = Clock::now();
    event.axis = JsAxis::kRX;
    if (az_chn.invert) az = -az;
    event.value = az;
    fsm_->GetContext().js_axis_queue->Push(event);
  }

  prev_msg = msg;
}

void SbotSystem::Run() {
  // start control thread
  control_thread_ = std::thread(&SbotSystem::ControlLoop, this);

  XLOG_INFO("SbotSystem: entering main loop");
  keep_main_loop_ = true;
  while (keep_main_loop_) {
    fsm_->Update();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  XLOG_INFO("SbotSystem: main loop exited");
}

void SbotSystem::Stop() {
  // stop the control loop
  keep_control_loop_ = false;
  if (control_thread_.joinable()) control_thread_.join();

  // stop the hid input
  if (config_.control_settings.user_input.type ==
      SbotConfig::UserInputType::kJoystick) {
    hid_event_listener_.reset();
    joystick_->Close();
  } else if (config_.control_settings.user_input.type ==
             SbotConfig::UserInputType::kRcReceiver) {
    sbus_rc_->Close();
  }

  // stop the robot
  sbot_->SetSteeringCommand({0, 0, 0, 0});
  sbot_->SetDrivingCommand({0, 0, 0, 0});

  // finally stop the main loop
  keep_main_loop_ = false;
}
}  // namespace xmotion
