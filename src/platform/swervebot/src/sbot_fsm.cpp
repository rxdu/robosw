/*
 * @file sbot_fsm.cpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "swervebot/sbot_fsm.hpp"

namespace xmotion {
OptionalStateVariant ModeTransition::Transit(ManualMode& state,
                                             ControlContext& context) {
  if (context.config.control_settings.user_input.type ==
      SbotConfig::UserInputType::kJoystick) {
    JsButton btn;
    if (context.js_button_queue->TryPop(btn)) {
      return AutoMode{context};
    }
  } else {
    LeverEvent mode_switch_event;
    if (context.rc_lever_queue->TryPop(mode_switch_event)) {
      if (mode_switch_event.value > 0) {
        return AutoMode{context};
      }
    }
  }
  return std::nullopt;
}

OptionalStateVariant ModeTransition::Transit(AutoMode& state,
                                             ControlContext& context) {
  if (context.config.control_settings.user_input.type ==
      SbotConfig::UserInputType::kJoystick) {
    JsButton btn;
    if (context.js_button_queue->TryPop(btn)) {
      return ManualMode{context};
    }
  } else {
    LeverEvent mode_switch_event;
    if (context.rc_lever_queue->TryPop(mode_switch_event)) {
      if (mode_switch_event.value < 0) {
        return ManualMode{context};
      }
    }
  }
  return std::nullopt;
}
}  // namespace xmotion
