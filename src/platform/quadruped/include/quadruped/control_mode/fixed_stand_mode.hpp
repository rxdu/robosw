/*
 * @file fixed_stand_mode.hpp
 * @date 7/4/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUADRUPED_MOTION_FIXED_STAND_MODE_HPP
#define QUADRUPED_MOTION_FIXED_STAND_MODE_HPP

#include "fsm/fsm_template.hpp"
#include "quadruped/control_context.hpp"
#include "time/stopwatch.hpp"

namespace xmotion {
class FixedStandMode : public FsmState<ControlContext> {
 public:
  FixedStandMode(const ControlContext& context);
  void Update(ControlContext& context);

 private:
  StopWatch sw_;
  QuadrupedModel::JointGains desired_gains_;
  QuadrupedModel::State initial_state_;
  QuadrupedModel::State target_state_;
};
}  // namespace xmotion

#endif  // QUADRUPED_MOTION_FIXED_STAND_MODE_HPP