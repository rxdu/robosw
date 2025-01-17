/*
 * @file ws_sbot_base.cpp
 * @date 11/13/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "swervebot/ws_sbot_base.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
WsSbotBase::WsSbotBase(const SbotConfig::BaseSettings& config)
    : config_(config) {}

bool WsSbotBase::Initialize() {
  steering_motor_ =
      std::make_shared<SmsStsServoArray>(config_.steering_motor_port);
  steering_motor_->SetPositionOffset(180);
  steering_motor_->SetDefaultPosition(0);
  steering_motor_->RegisterMotor(1);
  steering_motor_->RegisterMotor(2);
  steering_motor_->RegisterMotor(3);
  steering_motor_->RegisterMotor(4);
  if (!steering_motor_->Connect()) {
    std::cout << "Failed to connect to steering motor" << std::endl;
    return false;
  }

  // set up driving motors
  driving_motor_ = std::make_shared<Ddsm210Array>(config_.driving_motor_port);
  driving_motor_->RegisterMotor(1);
  driving_motor_->RegisterMotor(2);
  driving_motor_->RegisterMotor(3);
  driving_motor_->RegisterMotor(4);
  if (!driving_motor_->Connect()) {
    std::cout << "Failed to connect to driving motor" << std::endl;
    return false;
  }

  ModelConfig config;
  config.kinematics_param.track_width = sbot_track_width;
  config.kinematics_param.wheel_base = sbot_wheel_base;
  config.kinematics_param.wheel_radius = sbot_wheel_radius;
  
  config.kinematics_param.max_driving_speed = config_.max_driving_speed;
  config.kinematics_param.max_steering_angle = config_.max_steering_angle;
  config.kinematics_param.linear_vel_deadband = config_.driving_deadzone;
  config.kinematics_param.angular_vel_deadband = config_.steering_deadzone;

  config.steering_motors = steering_motor_;
  config.driving_motors = driving_motor_;
  config.reverse_right_wheels = true;
  robot_ = std::make_unique<SwerveDriveRobot>(config);

  return true;
}

void WsSbotBase::SetMotionCommand(const Twist& twist) {
  current_twist_ = twist;
}

void WsSbotBase::Update(double dt) { robot_->Update(current_twist_, dt); }

void WsSbotBase::SetSteeringCommand(const std::array<float, 4>& angles) {
  if (robot_ == nullptr) {
    std::cout << "Robot model is not initialized" << std::endl;
    return;
  }
  robot_->SetSteeringCommand(angles);
}

void WsSbotBase::SetDrivingCommand(const std::array<float, 4>& speeds) {
  if (robot_ == nullptr) {
    std::cout << "Robot model is not initialized" << std::endl;
    return;
  }
  robot_->SetDrivingCommand(speeds);
}
}  // namespace xmotion