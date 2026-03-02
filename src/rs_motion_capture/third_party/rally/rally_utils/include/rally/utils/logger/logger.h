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
#ifndef RALLY_UTILS_LOGGER_LOGGER_H
#define RALLY_UTILS_LOGGER_LOGGER_H

#include <vector>
#include <iostream>
#include "rally/utils/logger/common.h"
#include "rally/utils/logger/sinks/base_sink.h"
#include "rally/utils/logger/detail/log_msg.h"

namespace rally {
namespace log {

class Logger {
public:
    using Ptr = std::shared_ptr<Logger>;

    Logger(std::string name, std::vector<BaseSink::Ptr> sinks) : name_(name), sinks_(sinks) {}

    virtual ~Logger() = default;

    void log(Level lvl, const std::string& str) {
        log_(lvl, str);
    }

    void setLevel(Level log_level) {
        level_.store(static_cast<int>(log_level));
    }

    Level getLevel() const {
        return static_cast<Level>(level_.load(std::memory_order_relaxed));
    }

    const std::string &name() const {
        return name_;
    }

    std::vector<BaseSink::Ptr> &getSinks() {
        return sinks_;
    }

protected:
    RALLY_DISALLOW_COPY_AND_ASSIGN(Logger)

    // return true logging is enabled for the given level.
    bool shouldLog(Level msg_level) const {
        return static_cast<int>(msg_level) >= level_.load(std::memory_order_relaxed);
    }

    std::string name_;
    std::vector<BaseSink::Ptr> sinks_;
    level_t level_{static_cast<int>(Level::INFO)};

    // common implementation for after templated public api has been resolved
    void log_(Level lvl, const std::string& str) {
        bool log_enabled = shouldLog(lvl);
        if (!log_enabled) {
            return;
        }
        LogMsg log_msg(name_, lvl, str);
        log_it_(log_msg, log_enabled);
    }

    // log the given message (if the given log level is high enough),
    // and save backtrace (if backtrace is enabled).
    void log_it_(const LogMsg &log_msg, bool log_enabled) {
        if (log_enabled) {
            sink_it_(log_msg);
        }
    }
    virtual void sink_it_(const LogMsg &msg) {
        for (auto &sink : sinks_) {
            sink->log(msg);
        }
        flush_();
    }
    virtual void flush_() {
        for (auto &sink : sinks_) {
            sink->flush();
        }
    }
};

}  // namespace log
}  // namespace rally

#endif  // RALLY_UTILS_LOGGER_LOGGER_H
