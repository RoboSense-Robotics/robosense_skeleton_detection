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
#ifndef RALLY_UTILS_LOGGER_DETAIL_REGISTRY_H
#define RALLY_UTILS_LOGGER_DETAIL_REGISTRY_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <mutex>
#include "rally/utils/logger/common.h"
#include "rally/utils/logger/async_logger.h"

namespace rally {
namespace log {

class Registry {
public:
    void registerLogger(Logger::Ptr new_logger);

    Logger::Ptr get(const std::string &logger_name);

    Logger::Ptr getDefaultLogger();

    Logger *getDefaultRaw();

    // set default logger.
    // default logger is stored in default_logger_ (for faster retrieval) and in the loggers_ map.
    void setDefaultLogger(Logger::Ptr logger);

    void setTp(ThreadPool::Ptr tp);

    ThreadPool::Ptr getTp();

    std::recursive_mutex &getTpMutex() {
        return tp_mutex_;
    }

    void drop(const std::string &logger_name);

    void dropAll();

    void shutdown();

    static Registry &getInstance() {
        return Singleton<rally::log::Registry>::getInstance();
    }

    Registry();

    ~Registry();

private:

    std::recursive_mutex tp_mutex_;
    ThreadPool::Ptr tp_;

    std::mutex logger_map_mutex_;
    std::unordered_map<std::string, std::shared_ptr<Logger>> loggers_;
    std::shared_ptr<Logger> default_logger_;
};

}  // namespace log
}  // namespace rally

#endif  // RALLY_UTILS_LOGGER_DETAIL_REGISTRY_H
