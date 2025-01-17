/*
 * @file sbot_system.hpp
 * @date 11/15/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SBOT_SYSTEM_HPP
#define XMOTION_SBOT_SYSTEM_HPP

#include <atomic>
#include <memory>
#include <chrono>

#include "input_hid/joystick_handler.hpp"
#include "input_hid/hid_event_listener.hpp"
#include "input_sbus/sbus_receiver.hpp"

#include "swervebot/sbot_fsm.hpp"

namespace xmotion {
class SbotSystem {
 public:
  SbotSystem(const SbotConfig& config);
  ~SbotSystem() = default;

  // Run the application
  bool Initialize();
  void Run();
  void Stop();

 private:
  void ControlLoop();
  void OnJsButtonEvent(const JsButton& btn, const JxButtonEvent& event);
  void OnJsAxisEvent(const JsAxis& axis, const float& value);
  void OnSbusMsgReceived(const RcMessage& msg);

  SbotConfig config_;
  std::atomic<bool> keep_main_loop_{false};

  // joystick
  std::unique_ptr<JoystickHandler> joystick_;
  std::shared_ptr<HidEventListener> hid_event_listener_;

  // sbus rc
  std::shared_ptr<SbusReceiver> sbus_rc_;

  // robot base
  std::shared_ptr<WsSbotBase> sbot_;

  // control loop
  std::thread control_thread_;
  std::atomic<bool> keep_control_loop_{false};
  std::unique_ptr<SbotFsm> fsm_;
};
}  // namespace xmotion

#endif  // XMOTION_SBOT_SYSTEM_HPP