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

#include "rally/utils/timer/timer.h"
#include "rally/utils/details/throw.h"
#include <cmath>

namespace rally {

namespace {
std::atomic<uint64_t> global_timer_id = {0};
inline uint64_t generateTimerId() { return global_timer_id.fetch_add(1); }
}  // namespace

Timer::Timer(const TimerOption& opt) {
    RENSURE(opt.period > 0 && opt.period < TIMER_MAX_INTERVAL_MS);
    options_ = opt;
    timer_id_ = generateTimerId();
}

bool Timer::initTimerTask() {
    task_ = std::make_shared<TimerTask>(timer_id_);
    task_->interval_ms = options_.period;
    task_->next_fire_duration_ms = task_->interval_ms;
    if (options_.oneshot) {
        std::weak_ptr<TimerTask> task_weak_ptr = task_;
        task_->callback = [callback = this->options_.callback, task_weak_ptr]() {
            auto task = task_weak_ptr.lock();
            if (task) {
                std::lock_guard<std::mutex> lg(task->mutex);
                callback();
            }
        };
    } else {
        std::weak_ptr<TimerTask> task_weak_ptr = task_;
        task_->callback = [callback = this->options_.callback, task_weak_ptr]() {
            auto task = task_weak_ptr.lock();
            if (!task) {
                return;
            }
            std::lock_guard<std::mutex> lg(task->mutex);
            auto start = Time::getNow().toNanosecond();
            callback();
            auto end = Time::getNow().toNanosecond();
            uint64_t execute_time_ns = end - start;
            uint64_t execute_time_ms =
#if defined(__aarch64__)
            ::llround(static_cast<double>(execute_time_ns) / 1e6);
#else
            std::llround(static_cast<double>(execute_time_ns) / 1e6);
#endif
            if (task->last_execute_time_ns == 0) {
                task->last_execute_time_ns = start;
            } else {
                task->accumulated_error_ns +=
                start - task->last_execute_time_ns - task->interval_ms * 1000000;
            }
            task->last_execute_time_ns = start;
            if (execute_time_ms >= task->interval_ms) {
                task->next_fire_duration_ms = TIMER_RESOLUTION_MS;
            } else {
#if defined(__aarch64__)
                int64_t accumulated_error_ms = ::llround(
#else
                int64_t accumulated_error_ms = std::llround(
#endif
                static_cast<double>(task->accumulated_error_ns) / 1e6);
                if (static_cast<int64_t>(task->interval_ms - execute_time_ms -
                                         TIMER_RESOLUTION_MS) >= accumulated_error_ms) {
                    task->next_fire_duration_ms =
                    task->interval_ms - execute_time_ms - accumulated_error_ms;
                } else {
                    task->next_fire_duration_ms = TIMER_RESOLUTION_MS;
                }
            }
            TimingWheel::getInstance().addTask(task);
        };
    }
    return true;
}

void Timer::start() {
    if (!started_.exchange(true)) {
        if (initTimerTask()) {
            TimingWheel::getInstance().addTask(task_);
            RDEBUG << "start timer [" << task_->timer_id_ << "]";
        }
    }
}

void Timer::stop() {
    if (started_.exchange(false) && task_) {
        RDEBUG << "stop timer, the timer_id: " << timer_id_;
        // using a shared pointer to hold task_->mutex before task_ reset
        auto tmp_task = task_;
        {
            std::lock_guard<std::mutex> lg(tmp_task->mutex);
            task_.reset();
        }
    }
}

Timer::~Timer() {
    if (task_) {
        stop();
    }
}

}  // namespace rally
