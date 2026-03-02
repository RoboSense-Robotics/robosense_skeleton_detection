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
#ifndef RALLY_UTILS_LOGGER_ASYNC_LOGGER_H
#define RALLY_UTILS_LOGGER_ASYNC_LOGGER_H

#include "rally/utils/logger/logger.h"
#include "rally/utils/logger/detail/thread_pool.h"

namespace rally {
namespace log {

class AsyncLogger final : public std::enable_shared_from_this<AsyncLogger>, public Logger {
public:
    friend AsyncLogger::Ptr createAsyncLogger(const std::string& name, const std::vector<BaseSink::Ptr> &sink,
                                              AsyncOverflowPolicy overflow_policy);

private:
    friend class ThreadPool;
    AsyncLogger(std::string logger_name, const std::vector<BaseSink::Ptr> &sinks, std::weak_ptr<ThreadPool> tp,
    AsyncOverflowPolicy overflow_policy = AsyncOverflowPolicy::BLOCK)
    : Logger(std::move(logger_name), std::move(sinks))
    , thread_pool_(std::move(tp))
    , overflow_policy_(overflow_policy)
    {}

private:
    void sink_it_(const LogMsg &msg) override;
    void flush_() override;
    void backend_sink_it_(const LogMsg &incoming_log_msg);
    void backend_flush_();

    std::weak_ptr<ThreadPool> thread_pool_;
    AsyncOverflowPolicy overflow_policy_;
};

}  // namespace log
}  // namespace rally

#endif  // RALLY_UTILS_LOGGER_ASYNC_LOGGER_H
