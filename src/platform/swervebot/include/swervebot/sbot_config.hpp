/*
 * @file sbot_config.hpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SBOT_CONFIG_HPP
#define XMOTION_SBOT_CONFIG_HPP

#include <string>

namespace xmotion {
struct SbotConfig {
  struct BaseSettings {
    std::string steering_motor_port;
    std::string driving_motor_port;
    float max_driving_speed;
    float max_steering_angle;
    float driving_deadzone;
    float steering_deadzone;
  };

  enum class UserInputType {
    kNone,
    kJoystick,
    kRcReceiver,
  };

  struct RcChannelMap {
    int channel;
    float min;
    float neutral;
    float max;
    bool invert;
  };

  /////////////////////////////////////////////////////////////////////////////

  BaseSettings base_settings;

  struct {
    struct {
      UserInputType type;

      struct {
        std::string device;
      } joystick;

      struct {
        std::string port;

        struct {
          RcChannelMap mode;
          RcChannelMap linear_x;
          RcChannelMap linear_y;
          RcChannelMap angular_z;
        } mapping;
      } rc_receiver;
    } user_input;

    struct {
      float driving_scale;
      float steering_scale;
    } manual_mode;

    struct {
      float driving_scale;
      float steering_scale;
    } auto_mode;
  } control_settings;
};

bool LoadConfigFile(const std::string &file_path, SbotConfig *config);
}  // namespace xmotion

#endif  // XMOTION_SBOT_CONFIG_HPP