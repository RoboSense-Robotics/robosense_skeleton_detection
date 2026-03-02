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
#ifndef RALLY_UTILS_THREAD_CONSUMER_WORKER_H
#define RALLY_UTILS_THREAD_CONSUMER_WORKER_H

#include <condition_variable>
#include <functional>
#include <memory>
#include <thread>
#include <deque>
#include "rally/utils/logger/log.h"

namespace rally {

template<class Data>
class ConsumerWorker {
public:
    using Ptr = std::shared_ptr<ConsumerWorker>;

    explicit ConsumerWorker(const std::string &name = "") {
        this->name_ = name;
    }

    virtual void add(const Data &data) {
        std::unique_lock<std::mutex> lock(mutex_);
        queue_.emplace_back(data);
        condition_.notify_all();
    }

    ~ConsumerWorker() {
        RDEBUG << name() << ": thread worker is stopping!";
        stop();
        RDEBUG << name() << ": thread worker is stopped!";
    }

    void bind(const std::function<void(const Data &)> &func) {
        this->func_ = func;
    }

    void start() {
        run_flag_ = true;
        if (thread_ptr_ == nullptr) {
            thread_ptr_.reset(new std::thread(&ConsumerWorker::core, this));
        }
    }

    void stop() {
        if (thread_ptr_ != nullptr) {
            run_flag_ = false;
            condition_.notify_all();
            if (thread_ptr_->joinable()) {
                thread_ptr_->join();
            }
        }
    }

private:
    const std::string name() const {
        return name_ + "/Thread worker ";
    }

    void core() {
        while (run_flag_) {
            Data data;
            {
                std::unique_lock<std::mutex> lock(mutex_);

                if (queue_.empty()) {
                    condition_.wait(lock);
                    if (!run_flag_) {
                        continue;
                    }
                    if (queue_.empty()) {
                        continue;
                    }
                }
                data = queue_.back();
                queue_.clear();
            }
            func_(data);
            condition_.notify_all();
        }
    }

    std::unique_ptr<std::thread> thread_ptr_;
    std::mutex mutex_;
    std::condition_variable condition_;

    bool run_flag_ = false;

    std::deque<Data> queue_;
    std::function<void(const Data &)> func_;

    std::string name_;
};

}  // namespace rally

#endif  // RALLY_UTILS_THREAD_CONSUMER_WORKER_H
