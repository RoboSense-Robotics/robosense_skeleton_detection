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
#ifndef RALLY_UTILS_LOGGER_LOG_LOG_H
#define RALLY_UTILS_LOGGER_LOG_LOG_H

#include <iostream>
#include "rally/utils/logger/detail/registry.h"
#include "rally/utils/logger/logger.h"
#include "rally/utils/logger/stream_logger.h"

namespace rally {
namespace log {

AsyncLogger::Ptr createAsyncLogger(const std::string& name, const std::vector<BaseSink::Ptr> &sink,
                                   AsyncOverflowPolicy overflow_policy = AsyncOverflowPolicy::BLOCK);

Logger::Ptr createSyncLogger(const std::string &name, const std::vector<BaseSink::Ptr> &sinks);

inline Logger::Ptr getLoggerByName(const std::string &name) {
    return Registry::getInstance().get(name);
}

inline void registerLogger(const Logger::Ptr &logger) {
    Registry::getInstance().registerLogger(logger);
}

inline void dropLoggerByName(const std::string &name) {
    Registry::getInstance().drop(name);
}

inline Logger *getDefaultLoggerRaw() {
    return Registry::getInstance().getDefaultRaw();
}

inline Level getDefaultLoggerLevel() {
    return getDefaultLoggerRaw()->getLevel();
}

inline void setDefaultLogger(Logger::Ptr default_logger) {
    Registry::getInstance().setDefaultLogger(std::move(default_logger));
}

}  // namespace log

inline void setLogLevel(const log::Level& level) {
    rally::log::getLoggerByName("default")->setLevel(level);
}

}  // namespace rally

#define RTRACE (rally::log::StreamLogger().trace())
#define RDEBUG (rally::log::StreamLogger().debug())
#define RINFO (rally::log::StreamLogger().info())
#define RWARN (rally::log::StreamLogger().warning())
#define RERROR (rally::log::StreamLogger().error())


#endif  // RALLY_UTILS_LOGGER_LOG_LOG_H
