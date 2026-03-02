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

#include "rally/utils/logger/detail/registry.h"
#include "rally/utils/logger/logger.h"
#include "rally/utils/logger/sinks/terminal_sink.h"
#include "rally/utils/logger/sinks/file_sink.h"
#include "rally/utils/dump_catcher/logging.h"

namespace rally {
namespace log {

Registry::Registry() {
    std::vector<BaseSink::Ptr> sinks(2);
    sinks[0].reset(new TerminalSink(stdout));
    std::string log_file = "/tmp/rally.txt";
    rally::log::FileSink::Ptr file_sink_ptr(new rally::log::FileSink(log_file));
    sinks[1] = file_sink_ptr;

    const char *default_logger_name = "default";
    default_logger_.reset(new Logger(default_logger_name, sinks));
    default_logger_->setLevel(Level::DEBUG);
    loggers_[default_logger_name] = default_logger_;

    // register dump_catcher logger
    const char *dump_catcher = "dump_catcher";
    Logger::Ptr dump_logger(new Logger(dump_catcher, std::move(sinks)));
    dump_logger->setLevel(Level::ERROR);
    registerLogger(dump_logger);
    {
#ifdef DUMP_CATCHER_OPEN
        InstallFailureSignalHandler();
        // 默认捕捉 SIGSEGV 信号信息输出会输出到 stderr，可以通过下面的方法自定义输出方式：
        auto func = [dump_logger](const char *data, int size) {
            std::string str = std::string(data, size);
            dump_logger->log(Level::ERROR, str);
        };
        InstallFailureWriter(func);
# endif  // DUMP_CATCHER_OPEN
    }
}

Registry::~Registry() {
    shutdown();
}

void Registry::registerLogger(Logger::Ptr logger) {
    std::lock_guard<std::mutex> lock(logger_map_mutex_);
    auto logger_name = logger->name();
    if (loggers_.find(logger_name) != loggers_.end()) {
        drop(logger_name);
        if (default_logger_->name() == logger_name) {
            setDefaultLogger(logger);
        }
    }
    loggers_[logger_name] = std::move(logger);
}

Logger::Ptr Registry::get(const std::string &logger_name) {
    std::lock_guard<std::mutex> lock(logger_map_mutex_);
    auto found = loggers_.find(logger_name);
    return found == loggers_.end() ? nullptr : found->second;
}

Logger::Ptr Registry::getDefaultLogger() {
    std::lock_guard<std::mutex> lock(logger_map_mutex_);
    return default_logger_;
}

Logger *Registry::getDefaultRaw() {
    return default_logger_.get();
}

void Registry::setDefaultLogger(Logger::Ptr logger) {
    std::lock_guard<std::mutex> lock(logger_map_mutex_);
    // remove previous default logger from the map
    if (default_logger_ != nullptr) {
        loggers_.erase(default_logger_->name());
    }
    if (logger != nullptr) {
        loggers_[logger->name()] = logger;
    }
    default_logger_ = std::move(logger);
}

void Registry::setTp(ThreadPool::Ptr tp) {
    std::lock_guard<std::recursive_mutex> lock(tp_mutex_);
    tp_ = std::move(tp);
}

ThreadPool::Ptr Registry::getTp() {
    std::lock_guard<std::recursive_mutex> lock(tp_mutex_);
    return tp_;
}

void Registry::drop(const std::string &logger_name) {
    std::lock_guard<std::mutex> lock(logger_map_mutex_);
    loggers_.erase(logger_name);
    if (default_logger_ && default_logger_->name() == logger_name) {
        default_logger_.reset();
    }
}

void Registry::dropAll() {
    std::lock_guard<std::mutex> lock(logger_map_mutex_);
    loggers_.clear();
    default_logger_.reset();
}

void Registry::shutdown() {
    dropAll();
    {
        std::lock_guard<std::recursive_mutex> lock(tp_mutex_);
        tp_.reset();
    }
}

}  // namespace log
}  // namespace rally
