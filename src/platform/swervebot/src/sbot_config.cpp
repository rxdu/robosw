/*
 * @file sbot_config.cpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "swervebot/sbot_config.hpp"

#include "yaml-cpp/yaml.h"

#include "logging/xlogger.hpp"

namespace xmotion {
bool LoadConfigFile(const std::string& file_path, SbotConfig* config) {
  try {
    YAML::Node config_node = YAML::LoadFile(file_path);

    //------------------------- base settings -------------------------//
    config->base_settings.steering_motor_port =
        config_node["robot_base"]["steering_motor_port"].as<std::string>();
    config->base_settings.driving_motor_port =
        config_node["robot_base"]["driving_motor_port"].as<std::string>();
    config->base_settings.max_driving_speed =
        config_node["robot_base"]["max_driving_speed"].as<float>();
    config->base_settings.max_steering_angle =
        config_node["robot_base"]["max_steering_angle"].as<float>();
    config->base_settings.driving_deadzone =
        config_node["robot_base"]["driving_deadzone"].as<float>();
    config->base_settings.steering_deadzone =
        config_node["robot_base"]["steering_deadzone"].as<float>();

    //------------------------- control settings -------------------------//
    auto user_input_node = config_node["control"]["user_input"];
    std::string input_type = user_input_node["type"].as<std::string>();
    if (input_type == "joystick") {
      config->control_settings.user_input.type =
          SbotConfig::UserInputType::kJoystick;
    } else if (input_type == "rc_receiver") {
      config->control_settings.user_input.type =
          SbotConfig::UserInputType::kRcReceiver;
    } else {
      config->control_settings.user_input.type =
          SbotConfig::UserInputType::kNone;
    }

    // joystick
    auto joystick_node = user_input_node["joystick"];
    config->control_settings.user_input.joystick.device =
        joystick_node["device"].as<std::string>();

    // rc receiver
    auto rc_receiver_node = user_input_node["rc_receiver"];
    config->control_settings.user_input.rc_receiver.port =
        rc_receiver_node["port"].as<std::string>();
    {
      config->control_settings.user_input.rc_receiver.mapping.mode.channel =
          rc_receiver_node["mapping"]["mode"]["channel"].as<int>();
      config->control_settings.user_input.rc_receiver.mapping.mode.min =
          rc_receiver_node["mapping"]["mode"]["min"].as<float>();
      config->control_settings.user_input.rc_receiver.mapping.mode.neutral =
          rc_receiver_node["mapping"]["mode"]["neutral"].as<float>();
      config->control_settings.user_input.rc_receiver.mapping.mode.max =
          rc_receiver_node["mapping"]["mode"]["max"].as<float>();
      config->control_settings.user_input.rc_receiver.mapping.mode.invert =
          rc_receiver_node["mapping"]["mode"]["invert"].as<bool>();

      config->control_settings.user_input.rc_receiver.mapping.linear_x.channel =
          rc_receiver_node["mapping"]["linear_x"]["channel"].as<int>();
      config->control_settings.user_input.rc_receiver.mapping.linear_x.min =
          rc_receiver_node["mapping"]["linear_x"]["min"].as<float>();
      config->control_settings.user_input.rc_receiver.mapping.linear_x.neutral =
          rc_receiver_node["mapping"]["linear_x"]["neutral"].as<float>();
      config->control_settings.user_input.rc_receiver.mapping.linear_x.max =
          rc_receiver_node["mapping"]["linear_x"]["max"].as<float>();
      config->control_settings.user_input.rc_receiver.mapping.linear_x.invert =
          rc_receiver_node["mapping"]["linear_x"]["invert"].as<bool>();

      config->control_settings.user_input.rc_receiver.mapping.linear_y.channel =
          rc_receiver_node["mapping"]["linear_y"]["channel"].as<int>();
      config->control_settings.user_input.rc_receiver.mapping.linear_y.min =
          rc_receiver_node["mapping"]["linear_y"]["min"].as<float>();
      config->control_settings.user_input.rc_receiver.mapping.linear_y.neutral =
          rc_receiver_node["mapping"]["linear_y"]["neutral"].as<float>();
      config->control_settings.user_input.rc_receiver.mapping.linear_y.max =
          rc_receiver_node["mapping"]["linear_y"]["max"].as<float>();
      config->control_settings.user_input.rc_receiver.mapping.linear_y.invert =
          rc_receiver_node["mapping"]["linear_y"]["invert"].as<bool>();

      config->control_settings.user_input.rc_receiver.mapping.angular_z
          .channel =
          rc_receiver_node["mapping"]["angular_z"]["channel"].as<int>();
      config->control_settings.user_input.rc_receiver.mapping.angular_z.min =
          rc_receiver_node["mapping"]["angular_z"]["min"].as<float>();
      config->control_settings.user_input.rc_receiver.mapping.angular_z
          .neutral =
          rc_receiver_node["mapping"]["angular_z"]["neutral"].as<float>();
      config->control_settings.user_input.rc_receiver.mapping.angular_z.max =
          rc_receiver_node["mapping"]["angular_z"]["max"].as<float>();
      config->control_settings.user_input.rc_receiver.mapping.angular_z.invert =
          rc_receiver_node["mapping"]["angular_z"]["invert"].as<bool>();
    }

    // manual mode
    config->control_settings.manual_mode.driving_scale =
        config_node["control"]["manual_mode"]["driving_scale"].as<float>();
    config->control_settings.manual_mode.steering_scale =
        config_node["control"]["manual_mode"]["steering_scale"].as<float>();

    // auto mode
    config->control_settings.auto_mode.driving_scale =
        config_node["control"]["auto_mode"]["driving_scale"].as<float>();
    config->control_settings.auto_mode.steering_scale =
        config_node["control"]["auto_mode"]["steering_scale"].as<float>();

  } catch (YAML::BadFile& e) {
    XLOG_ERROR("ConfigLoader: failed to open config file {}: {}", file_path,
               e.what());
    return false;
  }
  return true;
}
}  // namespace xmotion