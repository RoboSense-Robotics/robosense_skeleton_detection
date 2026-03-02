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
#ifndef RALLY_UTILS_TIMER_TIMER_BUCKET_H
#define RALLY_UTILS_TIMER_TIMER_BUCKET_H

#include <list>
#include <memory>
#include <mutex>
#include "rally/utils/timer/timer_task.h"

namespace rally {

class TimerBucket {
public:
    void addTask(const std::shared_ptr<TimerTask> &task) {
        std::lock_guard<std::mutex> lock(mutex_);
        task_list_.push_back(task);
    }

    std::mutex &mutex() { return mutex_; }

    std::list<std::weak_ptr<TimerTask>> &task_list() { return task_list_; }

private:
    std::mutex mutex_;
    std::list<std::weak_ptr<TimerTask>> task_list_;
};

}  // namespace rally

#endif  // RALLY_UTILS_TIMER_TIMER_BUCKET_H
