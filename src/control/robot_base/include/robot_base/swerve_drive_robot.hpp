/*
 * @file swerve_drive_robot.hpp
 * @date 11/9/24
 * @brief 4-wheel swerve drive robot control class
 *
 * Assumptions:
 * Motors/servos are numbered in the following order:
 * 1. Front right
 * 2. Front left
 * 3. Rear left
 * 4. Rear right
 *
 * Robot command:
 * - Steering angles: in radian
 * - Driving speeds: in m/s
 *
 * It's assumed that the underlying steering motor driver uses degree as unit.
 *
 * Robot coordinate system:
 * x: forward
 * y: left
 * z: up
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SWERVE_DRIVE_ROBOT_HPP
#define XMOTION_SWERVE_DRIVE_ROBOT_HPP

#include <vector>
#include <array>
#include <memory>

#include "interface/type/geometry_types.hpp"
#include "interface/driver/motor_controller_array_interface.hpp"

#include "robot_base/kinematics/swerve_drive_kinematics.hpp"

namespace xmotion {
class SwerveDriveRobot {
 public:
  struct Config {
    // model parameters
    SwerveDriveKinematics::Param kinematics_param;

    // control parameters
    double max_steering_error = 3.14;
    double driving_limiting_scale = 0.5;

    // actuator configuration
    bool reverse_left_wheels = false;
    bool reverse_right_wheels = false;
    std::shared_ptr<MotorControllerArrayInterface> driving_motors;
    std::shared_ptr<MotorControllerArrayInterface> steering_motors;
  };

 public:
  SwerveDriveRobot(const Config &config);

  // public interface
  void Update(const Twist &twist, double dt);
  Odometry GetOdometry();
  void ResetOdometry();

  // low-level control
  void SetSteeringCommand(const std::array<float, 4> &angles);
  void SetDrivingCommand(const std::array<float, 4> &speeds);

 private:
  Config config_;
  SwerveDriveKinematics kinematics_;
  Odometry odom_;
  std::vector<float> driving_speeds_;
  std::vector<float> steering_angles_;
};
}  // namespace xmotion

#endif  // XMOTION_SWERVE_DRIVE_ROBOT_HPP