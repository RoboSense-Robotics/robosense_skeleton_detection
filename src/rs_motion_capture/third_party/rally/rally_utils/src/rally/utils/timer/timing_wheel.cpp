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

#include "rally/utils/timer/timing_wheel.h"
#include "rally/utils/timer/rate.h"
#include <cmath>

namespace rally {

void TimingWheel::start() {
    std::lock_guard<std::mutex> lock(running_mutex_);
    if (!running_) {
        running_ = true;
        tick_thread_ = std::thread([this]() { this->tickFunc(); });
    }
}

void TimingWheel::shutdown() {
    std::lock_guard<std::mutex> lock(running_mutex_);
    if (running_) {
        running_ = false;
        if (tick_thread_.joinable()) {
            tick_thread_.join();
        }
    }
}

void TimingWheel::tick() {
    auto& bucket = work_wheel_[current_work_wheel_index_];
    {
        std::lock_guard<std::mutex> lock(bucket.mutex());
        auto ite = bucket.task_list().begin();
        while (ite != bucket.task_list().end()) {
            auto task = ite->lock();
            if (task) {
                auto* callback =
                reinterpret_cast<std::function<void()>*>(&(task->callback));
                std::async([this, callback] {
                    if (this->running_) {
                        (*callback)();
                    }
                });
            }
            ite = bucket.task_list().erase(ite);
        }
    }
}

void TimingWheel::addTask(const std::shared_ptr<TimerTask>& task) {
    addTask(task, current_work_wheel_index_);
}

void TimingWheel::addTask(const std::shared_ptr<TimerTask>& task,
                          const uint64_t current_work_wheel_index) {
    if (!running_) {
        start();
    }
    auto work_wheel_index = current_work_wheel_index +
                            static_cast<uint64_t>(std::ceil(
                            static_cast<double>(task->next_fire_duration_ms) /
                            TIMER_RESOLUTION_MS));
    if (work_wheel_index >= WORK_WHEEL_SIZE) {
        auto real_work_wheel_index = getWorkWheelIndex(work_wheel_index);
        task->remainder_interval_ms = real_work_wheel_index;
        auto assistant_ticks = work_wheel_index / WORK_WHEEL_SIZE;
        if (assistant_ticks == 1 &&
            real_work_wheel_index < current_work_wheel_index_) {
            work_wheel_[real_work_wheel_index].addTask(task);
        } else {
            auto assistant_wheel_index = 0;
            {
                std::lock_guard<std::mutex> lock(current_assistant_wheel_index_mutex_);
                assistant_wheel_index = getAssistantWheelIndex(
                current_assistant_wheel_index_ + assistant_ticks);
                assistant_wheel_[assistant_wheel_index].addTask(task);
            }
        }
    } else {
        work_wheel_[work_wheel_index].addTask(task);
    }
}

void TimingWheel::cascade(const uint64_t assistant_wheel_index) {
    auto& bucket = assistant_wheel_[assistant_wheel_index];
    std::lock_guard<std::mutex> lock(bucket.mutex());
    auto ite = bucket.task_list().begin();
    while (ite != bucket.task_list().end()) {
        auto task = ite->lock();
        if (task) {
            work_wheel_[task->remainder_interval_ms].addTask(task);
        }
        ite = bucket.task_list().erase(ite);
    }
}

void TimingWheel::tickFunc() {
    Rate rate(TIMER_RESOLUTION_MS * 1000000);  // ms to ns
    while (running_) {
        tick();
        // AINFO_EVERY(1000) << "Tick " << TickCount();
        tick_count_++;
        rate.sleep();
        {
            std::lock_guard<std::mutex> lock(current_work_wheel_index_mutex_);
            current_work_wheel_index_ =
            getWorkWheelIndex(current_work_wheel_index_ + 1);
        }
        if (current_work_wheel_index_ == 0) {
            {
                std::lock_guard<std::mutex> lock(current_assistant_wheel_index_mutex_);
                current_assistant_wheel_index_ =
                getAssistantWheelIndex(current_assistant_wheel_index_ + 1);
            }
            cascade(current_assistant_wheel_index_);
        }
    }
}

}  // namespace rally
