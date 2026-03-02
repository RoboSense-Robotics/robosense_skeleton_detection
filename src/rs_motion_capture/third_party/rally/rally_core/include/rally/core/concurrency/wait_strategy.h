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
#ifndef RALLY_CORE_CONCURRENCY_WAIT_STRATEGY_H
#define RALLY_CORE_CONCURRENCY_WAIT_STRATEGY_H

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <mutex>
#include <thread>

namespace rally {

class WaitStrategy {
public:
    virtual void notifyOne() {}

    virtual void breakAllWait() {}

    virtual bool emptyWait() = 0;

    virtual ~WaitStrategy() {}
};

class BlockWaitStrategy : public WaitStrategy {
public:
    BlockWaitStrategy() {}

    void notifyOne() override { cv_.notify_one(); }

    bool emptyWait() override {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock);
        return true;
    }

    void breakAllWait() override { cv_.notify_all(); }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
};

class SleepWaitStrategy : public WaitStrategy {
public:
    SleepWaitStrategy() {}

    explicit SleepWaitStrategy(uint64_t sleep_time_us)
    : sleep_time_us_(sleep_time_us) {}

    bool emptyWait() override {
        std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_us_));
        return true;
    }

    void setSleepTimeMicroSeconds(uint64_t sleep_time_us) {
        sleep_time_us_ = sleep_time_us;
    }

private:
    uint64_t sleep_time_us_ = 10000;
};

}  // namespace rally

#endif  // RALLY_CORE_CONCURRENCY_WAIT_STRATEGY_H
