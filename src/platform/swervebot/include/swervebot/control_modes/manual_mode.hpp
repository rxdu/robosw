/*
 * @file manual_mode.hpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_MANUAL_MODE_HPP
#define XMOTION_MANUAL_MODE_HPP

#include "fsm/fsm_template.hpp"
#include "swervebot/control_context.hpp"

namespace xmotion {
class ManualMode : public FsmState<ControlContext> {
 public:
  ManualMode(const ControlContext& context);
  void Update(ControlContext& context);

 private:
  TimePoint last_cmd_update_time_;
  float vx_{0.0};
  float vy_{0.0};
  float wz_{0.0};
};
}  // namespace xmotion

#endif  // XMOTION_MANUAL_MODE_HPP