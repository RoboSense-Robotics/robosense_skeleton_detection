/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#ifndef RALLY_UTILS_LOGGER_DETAIL_LOG_MSG_H
#define RALLY_UTILS_LOGGER_DETAIL_LOG_MSG_H

#include <string>
#include "rally/utils/logger/common.h"
#include "rally/utils/logger/detail/os.h"

namespace rally {
namespace log {

struct LogMsg {
    LogMsg() = default;

    LogMsg(log_clock::time_point log_time, std::string logger_name, Level lvl, std::string msg)
    : log_time_(log_time), logger_name_(logger_name), level_(lvl), msg_(msg),
      thread_id(os::thread_id()) {}

    LogMsg(std::string logger_name, Level lvl, std::string msg) : LogMsg(os::now(), logger_name, lvl, msg) {}

    log_clock::time_point log_time_;
    std::string logger_name_;
    Level level_{Level::DEBUG};
    std::string msg_;
    size_t thread_id{0};

};

}  // namespace log
}  // namespace rally

#endif  // RALLY_UTILS_LOGGER_DETAIL_LOG_MSG_H
