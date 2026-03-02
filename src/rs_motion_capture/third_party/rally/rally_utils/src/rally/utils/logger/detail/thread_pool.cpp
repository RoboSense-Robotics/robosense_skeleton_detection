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

#include <cassert>
#include <iostream>
#include "rally/utils/logger/detail/thread_pool.h"
#include "rally/utils/logger/common.h"
#include "rally/utils/logger/async_logger.h"

namespace rally {
namespace log {

ThreadPool::ThreadPool(size_t q_max_items, size_t threads_n, std::function<void()> on_thread_start)
: q_(q_max_items) {
    if (threads_n == 0 || threads_n > 1000) {
        throw "log::ThreadPool: invalid threads_n param (valid range is 1-1000)";
    }
    for (size_t i = 0; i < threads_n; i++) {
        threads_.emplace_back([this, on_thread_start] {
            on_thread_start();
            this->ThreadPool::worker_loop_();
        });
    }
}

ThreadPool::ThreadPool(size_t q_max_items, size_t threads_n)
: ThreadPool(q_max_items, threads_n, [] {}) {}

// message all threads to terminate gracefully join them
ThreadPool::~ThreadPool() {
    try {
        for (size_t i = 0; i < threads_.size(); i++) {
            post_async_msg_(AsyncMsg(AsyncMsgType::TERMINATE), AsyncOverflowPolicy::BLOCK);
        }

        for (auto &t : threads_) {
            t.join();
        }
    }
    catch (const std::exception &) {}
}

void ThreadPool::post_log(AsyncLoggerPtr &&worker_ptr, const LogMsg &msg, AsyncOverflowPolicy overflow_policy) {
    AsyncMsg async_m(std::move(worker_ptr), AsyncMsgType::LOG, msg);
    post_async_msg_(std::move(async_m), overflow_policy);
}

void ThreadPool::post_flush(AsyncLoggerPtr &&worker_ptr, AsyncOverflowPolicy overflow_policy) {
    post_async_msg_(AsyncMsg(std::move(worker_ptr), AsyncMsgType::FLUSH), overflow_policy);
}

size_t ThreadPool::overrun_counter() {
    return q_.overrun_counter();
}

size_t ThreadPool::queue_size() {
    return q_.size();
}

void ThreadPool::post_async_msg_(AsyncMsg &&new_msg, AsyncOverflowPolicy overflow_policy) {
    if (overflow_policy == AsyncOverflowPolicy::BLOCK) {
        q_.enqueue(std::move(new_msg));
    } else {
        q_.enqueue_nowait(std::move(new_msg));
    }
}

void ThreadPool::worker_loop_() {
    while (process_next_msg_()) {}
}

// process next message in the queue
// return true if this thread should still be active (while no terminate msg
// was received)
bool ThreadPool::process_next_msg_() {
    AsyncMsg incoming_async_msg;
    bool dequeued = q_.dequeue_for(incoming_async_msg, std::chrono::seconds(10));
    if (!dequeued) {
        return true;
    }

    switch (incoming_async_msg.msg_type) {
        case AsyncMsgType::LOG: {
            incoming_async_msg.worker_ptr->backend_sink_it_(incoming_async_msg);
            return true;
        }
        case AsyncMsgType::FLUSH: {
            incoming_async_msg.worker_ptr->backend_flush_();
            return true;
        }

        case AsyncMsgType::TERMINATE: {
            return false;
        }

        default: {
            assert(false);
        }
    }

    return true;
}

}  // namespace log
}  // namespace rally
