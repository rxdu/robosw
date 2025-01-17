/*
 * @file swerve_drive_robot.cpp
 * @date 11/9/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "robot_base/swerve_drive_robot.hpp"

#include "logging/xlogger.hpp"

namespace xmotion {
SwerveDriveRobot::SwerveDriveRobot(const SwerveDriveRobot::Config& config)
    : config_(config), kinematics_(config.kinematics_param) {}

void SwerveDriveRobot::Update(const Twist& twist, double dt) {
  //-------------------- update actuator commands --------------------//
  Twist twist_filtered = twist;
  if (std::abs(twist.linear.x()) <
      config_.kinematics_param.linear_vel_deadband) {
    twist_filtered.linear.x() = 0;
  }
  if (std::abs(twist.linear.y()) <
      config_.kinematics_param.linear_vel_deadband) {
    twist_filtered.linear.y() = 0;
  }
  if (std::abs(twist.angular.z()) <
      config_.kinematics_param.angular_vel_deadband) {
    twist_filtered.angular.z() = 0;
  }

  // XLOG_INFO_STREAM("Input: vx = " << twist_filtered.linear.x()
  //                                 << ", vy = " << twist_filtered.linear.y()
  //                                 << ", wz = " <<
  //                                 twist_filtered.angular.z());

  auto cmd = kinematics_.ComputeWheelCommands(twist_filtered);
  // XLOG_INFO_STREAM("Output: ");
  // for (int i = 0; i < 4; ++i) {
  //   XLOG_INFO_STREAM("Wheel " << i << ": speed = " << cmd.speeds[i]
  //                             << ", angle = " << cmd.angles[i]);
  // }

  //------------------------- update odometry -------------------------//
  config_.driving_motors->GetSpeeds(driving_speeds_);
  // convert speed rpm to m/s
  for (int i = 0; i < 4; ++i) {
    driving_speeds_[i] = driving_speeds_[i] * 2 * M_PI *
                         config_.kinematics_param.wheel_radius / 60;
  }
  config_.steering_motors->GetPositions(steering_angles_);
  // convert angle degree to radian
  for (int i = 0; i < 4; ++i) {
    steering_angles_[i] = steering_angles_[i] / 180.0f * M_PI;
  }

  // XLOG_INFO_STREAM("Motor actual speeds: "
  //                  << driving_speeds_[0] << ", " << driving_speeds_[1] << ",
  //                  "
  //                  << driving_speeds_[2] << ", " << driving_speeds_[3]);
  // XLOG_INFO_STREAM("Servo actual angles: "
  //                  << steering_angles_[0] << ", " << steering_angles_[1] <<
  //                  ", "
  //                  << steering_angles_[2] << ", " << steering_angles_[3]);

  //------------------- regulate cmd based on feedback ----------------//

  double angle_errors[4] = {0, 0, 0, 0};
  for (int i = 0; i < 4; ++i) {
    angle_errors[i] = cmd.angles[i] - steering_angles_[i];
    if (angle_errors[i] > M_PI) {
      angle_errors[i] -= 2 * M_PI;
    } else if (angle_errors[i] < -M_PI) {
      angle_errors[i] += 2 * M_PI;
    }
  }
  double max_error = *std::max_element(angle_errors, angle_errors + 4);

  // XLOG_INFO_STREAM("Angle errors: " << angle_errors[0] << ", "
  //                                   << angle_errors[1] << ", "
  //                                   << angle_errors[2] << ", "
  //                                   << angle_errors[3] << " => " <<
  //                                   max_error);
  // double speed_factor = 1.0 - max_error / config_.max_steering_error *
  //                                 config_.driving_limiting_scale;
  double speed_factor = 1.0 / (1.0 + std::exp(60 * (max_error - 0.145)));
  if (speed_factor > 1.0) speed_factor = 1.0;
  if (speed_factor < 0.0) speed_factor = 0.0;
  XLOG_INFO_STREAM("Speed factor: " << speed_factor);
  for (int i = 0; i < 4; ++i) {
    cmd.speeds[i] *= speed_factor;
  }

  SetSteeringCommand(cmd.angles);
  SetDrivingCommand(cmd.speeds);

  // XLOG_INFO("--------------------");
}

Odometry SwerveDriveRobot::GetOdometry() { return odom_; }

void SwerveDriveRobot::ResetOdometry() {
  odom_.pose = {{0, 0, 0}, {1, 0, 0, 0}};
  odom_.twist = {{0, 0, 0}, {0, 0, 0}};
}

void SwerveDriveRobot::SetSteeringCommand(const std::array<float, 4>& angles) {
  if (config_.steering_motors == nullptr) {
    XLOG_ERROR("Steering motor group is not initialized");
    return;
  }
  // set steering motors angle
  std::vector<float> steering_angles;
  for (int i = 0; i < 4; ++i) {
    steering_angles.push_back(angles[i] / M_PI * 180.0f);
  }

//   XLOG_INFO_STREAM("Desired servo angles: "
//                    << steering_angles[0] << ", " << steering_angles[1] << ", "
//                    << steering_angles[2] << ", " << steering_angles[3]);
  config_.steering_motors->SetPositions(steering_angles);
}

void SwerveDriveRobot::SetDrivingCommand(const std::array<float, 4>& speeds) {
  if (config_.driving_motors == nullptr) {
    XLOG_ERROR("Driving motor group is not initialized");
    return;
  }
  // set driving motors speed
  std::vector<float> driving_speeds;
  for (int i = 0; i < 4; ++i) {
    // convert speed m/s to rpm
    float rpm =
        speeds[i] / (2 * M_PI * config_.kinematics_param.wheel_radius) * 60;
    if (config_.reverse_left_wheels && (i == 1 || i == 2)) {
      driving_speeds.push_back(-rpm);
    } else if (config_.reverse_right_wheels && (i == 0 || i == 3)) {
      driving_speeds.push_back(-rpm);
    } else {
      driving_speeds.push_back(rpm);
    }
  }
//   XLOG_INFO_STREAM("Desired motor speeds: "
//                    << driving_speeds[0] << ", " << driving_speeds[1] << ", "
//                    << driving_speeds[2] << ", " << driving_speeds[3]);
  config_.driving_motors->SetSpeeds(driving_speeds);
}
}  // namespace xmotion