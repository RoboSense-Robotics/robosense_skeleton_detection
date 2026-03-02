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
#ifndef RALLY_UTILS_LOGGER_DETAIL_THREAD_POOL_H
#define RALLY_UTILS_LOGGER_DETAIL_THREAD_POOL_H

#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <functional>

#include "rally/utils/logger/detail/log_msg.h"
#include "rally/utils/logger/detail/mpmc_blocking_queue.h"
#include "rally/utils/logger/detail/os.h"

namespace rally {
namespace log {

class AsyncLogger;
using AsyncLoggerPtr = std::shared_ptr<AsyncLogger>;

enum class AsyncOverflowPolicy {
    BLOCK = 0,
    OVERRUN = 1,
};

enum class AsyncMsgType {
    LOG = 0,
    FLUSH = 1,
    TERMINATE = 2,
};

struct AsyncMsg : LogMsg {
    AsyncMsgType msg_type{AsyncMsgType::LOG};
    AsyncLoggerPtr worker_ptr;

    AsyncMsg() = default;
    ~AsyncMsg() = default;

    // should only be moved in or out of the queue..
    AsyncMsg(const AsyncMsg &) = delete;

    AsyncMsg(AsyncMsg &&) = default;
    AsyncMsg &operator=(AsyncMsg &&) = default;

    // construct from log_msg with given type
    AsyncMsg(AsyncLoggerPtr &&worker, AsyncMsgType the_type, const LogMsg &m)
    : LogMsg{m}
    , msg_type{the_type}
    , worker_ptr{std::move(worker)}
    {}

    AsyncMsg(AsyncLoggerPtr &&worker, AsyncMsgType the_type)
    : LogMsg{}
    , msg_type{the_type}
    , worker_ptr{std::move(worker)}
    {}

    explicit AsyncMsg(AsyncMsgType the_type)
    : AsyncMsg{nullptr, the_type}
    {}
};

class ThreadPool {
public:
    using Ptr = std::shared_ptr<ThreadPool>;
    using ItemType = AsyncMsg;
    using QType = MPMCBlcokingQueue<ItemType>;

    ThreadPool(size_t q_max_items, size_t threads_n, std::function<void()> on_thread_start);
    ThreadPool(size_t q_max_items, size_t threads_n);

    // message all threads to terminate gracefully join them
    ~ThreadPool();

    ThreadPool(const ThreadPool &) = delete;
    ThreadPool &operator=(ThreadPool &&) = delete;

    void post_log(AsyncLoggerPtr &&worker_ptr, const LogMsg &msg, AsyncOverflowPolicy overflow_policy);
    void post_flush(AsyncLoggerPtr &&worker_ptr, AsyncOverflowPolicy overflow_policy);
    size_t overrun_counter();
    size_t queue_size();
    void get_threads(std::vector<std::thread*>& threads) {
        threads.resize(threads_.size());
        for (size_t i = 0; i < threads_.size(); ++i) {
            threads[i] = &threads_[i];
        }
    }

private:
    QType q_;

    std::vector<std::thread> threads_;

    void post_async_msg_(AsyncMsg &&new_msg, AsyncOverflowPolicy overflow_policy);
    void worker_loop_();

    // process next message in the queue
    // return true if this thread should still be active (while no terminate msg
    // was received)
    bool process_next_msg_();
};

}  // namespace log
}  // namespace rally

#endif  // RALLY_UTILS_LOGGER_DETAIL_THREAD_POOL_H
