/*
 * @file test_sbus_receiver.cpp
 * @date 11/23/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <csignal>
#include <thread>
#include <chrono>
#include <iostream>

#include "input_sbus/sbus_receiver.hpp"

using namespace xmotion;

bool keep_running = true;

void OnSbusMsgReceived(const RcMessage &msg) {
  std::cout << "Sbus message received: " << std::endl;
  for (int i = 0; i < 18; ++i) {
    std::cout << "ch" << i << ": " << msg.channels[i] << " ";
  }
  if (msg.frame_loss) std::cout << "\tFrame lost";
  if (msg.fault_protection) std::cout << "\tFault protection";
  std::cout << std::endl;
}

int main(int argc, char **argv) {
  std::signal(SIGINT, [](int signum) { keep_running = false; });

  SbusReceiver sbus_receiver("/dev/ttyS4");
  sbus_receiver.SetOnRcMessageReceivedCallback(OnSbusMsgReceived);

  if (!sbus_receiver.Open()) {
    std::cout << "failed to open sbus receiver" << std::endl;
    return -1;
  }

  while (keep_running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
