/* 
 * messenger.hpp
 *
 * Created on 4/4/22 9:52 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_APPS_TBOT_INCLUDE_MESSENGER_HPP
#define ROBOSW_SRC_APPS_TBOT_INCLUDE_MESSENGER_HPP

#include <memory>
#include <unordered_map>

#include "interface/types.hpp"
#include "async_port/async_can.hpp"
#include "imview/data_buffer.hpp"

namespace robosw {
class Messenger {
 public:
  enum class DataBufferIndex : RSEnumBaseType {
    kRawRpmLeft = 0,
    kRawRpmRight,
    kFilteredRpmLeft,
    kFilteredRpmRight
  };

 public:
  Messenger(RSTimePoint tp);

  bool Start(std::string can);
  void Stop();
  bool IsStarted();

  void SendPwmCommand(float left, float right);
  swviz::DataBuffer &GetDataBuffer(DataBufferIndex idx);

 private:
  RSTimePoint t0_;
  std::shared_ptr<AsyncCAN> can_ = nullptr;

  std::unordered_map<DataBufferIndex, swviz::DataBuffer> rpm_buffers_;

  void HandleCanFrame(can_frame *rx_frame);
};
}

#endif //ROBOSW_SRC_APPS_TBOT_INCLUDE_MESSENGER_HPP