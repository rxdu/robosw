/* 
 * tbot_context.hpp
 *
 * Created on 4/4/22 9:38 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_APPS_TBOT_INCLUDE_TBOT_CONTEXT_HPP
#define ROBOSW_SRC_APPS_TBOT_INCLUDE_TBOT_CONTEXT_HPP

#include <memory>

#include "interface/types.hpp"
#include "tbot/messenger.hpp"

namespace robosw {
struct TbotContext {
  float plot_history = 20.0;
  RSTimePoint time_of_start;
  std::shared_ptr<Messenger> msger = nullptr;
};
}

#endif //ROBOSW_SRC_APPS_TBOT_INCLUDE_TBOT_CONTEXT_HPP