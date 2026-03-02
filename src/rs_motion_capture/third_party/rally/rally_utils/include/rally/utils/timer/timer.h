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
#ifndef RALLY_UTILS_TIMER_TIMER_H
#define RALLY_UTILS_TIMER_TIMER_H

#include <atomic>
#include <memory>
#include "rally/utils/timer/time.h"
#include "rally/utils/timer/rate.h"
#include "rally/utils/timer/timer_task.h"
#include "rally/utils/timer/timing_wheel.h"

namespace rally {

struct TimerOption {
    /// @brief The period of the timer, unit is ms, min: 1
    uint32_t period{1};

    /// @brief The task that the timer needs to perform
    std::function<void()> callback;

    /// @brief True: perform the callback only after the first timing cycle
    // False: perform the callback every timed period
    bool oneshot{true};
};

class Timer {
public:
    using Ptr = std::shared_ptr<Timer>;

    Timer(const TimerOption &options = TimerOption());

    ~Timer();

    void start();

    /// @brief Stop the timer
    void stop();

private:
    bool initTimerTask();

    uint64_t timer_id_;
    TimerOption options_;
    std::shared_ptr<TimerTask> task_;
    std::atomic<bool> started_ = {false};
};

}  // namespace rally

#endif  // RALLY_UTILS_TIMER_TIMER_H
