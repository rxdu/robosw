/*
 * @file manual_mode.cpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "swervebot/control_modes/manual_mode.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
ManualMode::ManualMode(const ControlContext& context) {
  XLOG_INFO("==> Switched to ManualMode");
  last_cmd_update_time_ = Clock::now();
}

void ManualMode::Update(ControlContext& context) {
  // handle joystick events
  AxisEvent axis_event;
  while (context.js_axis_queue->TryPop(axis_event)) {
    last_cmd_update_time_ = Clock::now();
    if (axis_event.axis == JsAxis::kX) {
      vy_ = -axis_event.value;
    } else if (axis_event.axis == JsAxis::kY) {
      vx_ = -axis_event.value;
    } else if (axis_event.axis == JsAxis::kRX) {
      wz_ = -axis_event.value;
    }
  }
  if (Clock::now() - last_cmd_update_time_ > std::chrono::milliseconds(500)) {
    vx_ = 0.0;
    vy_ = 0.0;
    wz_ = 0.0;
  }

  // XLOG_INFO_STREAM("====> ManualMode: vx = " << vx_ << ", vy = " << vy_
  //                                            << ", wz = " << wz_);

  // convert from [-1,1] to actual speed and steering angle
  UserCommand cmd;
  cmd.vx = vx_ * context.config.control_settings.manual_mode.driving_scale;
  cmd.vy = vy_ * context.config.control_settings.manual_mode.driving_scale;
  cmd.vy *= 0.5;
  if(cmd.vy < 0.1 && cmd.vy > -0.1) cmd.vy = 0; 
  cmd.wz = wz_ * context.config.control_settings.manual_mode.steering_scale;

  //  command_queue
  context.command_queue->Push(cmd);
}
}  // namespace xmotion
