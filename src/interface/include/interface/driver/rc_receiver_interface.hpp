/*
 * @file rc_interface.hpp
 * @date 11/23/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */
#ifndef RC_RECEIVER_INTERFACE_HPP
#define RC_RECEIVER_INTERFACE_HPP

#include "interface/type/base_types.hpp"

#include <array>

namespace xmotion {
struct RcMessage {
  std::array<float, 24> channels = {0};
  bool frame_loss = false;
  bool fault_protection = false;
};

class RcReceiverInterface {
 public:
  virtual ~RcReceiverInterface() = default;

  // Public API
  virtual bool Open() = 0;
  virtual void Close() = 0;
  virtual bool IsOpened() const = 0;

  /// @brief Scale the channel value to [-1, 1]
  static float ScaleChannelValue(float value, float min, float neutral,
                                 float max) {
    if (value < neutral) {
      return (value - neutral) / (neutral - min);
    } else {
      return (value - neutral) / (max - neutral);
    }
  }

  using OnRcMessageReceivedCallback = std::function<void(const RcMessage&)>;
  virtual void SetOnRcMessageReceivedCallback(
      OnRcMessageReceivedCallback cb) = 0;
};
}  // namespace xmotion

#endif  // RC_RECEIVER_INTERFACE_HPP
