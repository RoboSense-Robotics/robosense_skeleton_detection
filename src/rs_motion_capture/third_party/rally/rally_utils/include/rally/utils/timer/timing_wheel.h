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
#ifndef RALLY_UTILS_TIMER_TIMING_WHEEL_H
#define RALLY_UTILS_TIMER_TIMING_WHEEL_H

#include <future>
#include <list>
#include <memory>
#include <thread>
#include "rally/utils/timer/timer_bucket.h"
#include "rally/utils/details/macros.h"

namespace rally {

struct TimerTask;

static const uint64_t WORK_WHEEL_SIZE = 512;
static const uint64_t ASSISTANT_WHEEL_SIZE = 64;
static const uint64_t TIMER_RESOLUTION_MS = 1;
static const uint64_t TIMER_MAX_INTERVAL_MS =
WORK_WHEEL_SIZE * ASSISTANT_WHEEL_SIZE * TIMER_RESOLUTION_MS;

class TimingWheel {
public:
    TimingWheel() {}

    ~TimingWheel() {
        if (running_) {
            shutdown();
        }
    }

    void start();

    void shutdown();

    void tick();

    void addTask(const std::shared_ptr<TimerTask> &task);

    void addTask(const std::shared_ptr<TimerTask> &task,
                 const uint64_t current_work_wheel_index);

    void cascade(const uint64_t assistant_wheel_index);

    void tickFunc();

    uint64_t getTickCount() const { return tick_count_; }

    static TimingWheel &getInstance() noexcept {
        return rally::Singleton<rally::TimingWheel>::getInstance();
    }

private:
    uint64_t getWorkWheelIndex(const uint64_t index) {
        return index & (WORK_WHEEL_SIZE - 1);
    }

    uint64_t getAssistantWheelIndex(const uint64_t index) {
        return index & (ASSISTANT_WHEEL_SIZE - 1);
    }

    bool running_ = false;
    uint64_t tick_count_ = 0;
    std::mutex running_mutex_;
    TimerBucket work_wheel_[WORK_WHEEL_SIZE];
    TimerBucket assistant_wheel_[ASSISTANT_WHEEL_SIZE];
    uint64_t current_work_wheel_index_ = 0;
    std::mutex current_work_wheel_index_mutex_;
    uint64_t current_assistant_wheel_index_ = 0;
    std::mutex current_assistant_wheel_index_mutex_;
    std::thread tick_thread_;
};

}  // namespace rally

#endif  // RALLY_UTILS_TIMER_TIMING_WHEEL_H
