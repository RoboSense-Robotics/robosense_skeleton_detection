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

#include "rally/utils/thread/thread_worker.h"

namespace rally {

void ThreadWorker::start() noexcept {
    if (thread_ptr_ == nullptr) {
        thread_ptr_ = std::make_unique<std::thread>(&ThreadWorker::core, this);
    }
    const std::lock_guard<std::mutex> lock{mutex_};
    work_flag_ = false;
    exit_flag_ = false;
}


void ThreadWorker::stop() noexcept {
    if (thread_ptr_ == nullptr) {
        return;
    }
    {
        const std::lock_guard<std::mutex> lock{mutex_};
        work_flag_ = true;
        exit_flag_ = true;
    }
    condition_.notify_one();
    thread_ptr_->join();
    thread_ptr_.reset(nullptr);
}

void ThreadWorker::wakeUp() noexcept {
    {
        const std::lock_guard<std::mutex> lock{mutex_};
        work_flag_ = true;
    }
    condition_.notify_one();
}

void ThreadWorker::join() noexcept {
    std::unique_lock<std::mutex> lock{mutex_};
    condition_.wait(lock, [&]() noexcept { return !work_flag_; });
}

void ThreadWorker::core() noexcept {
    while (true) {
        {
            std::unique_lock<std::mutex> lock{mutex_};
            condition_.wait(lock, [&]() noexcept { return work_flag_; });
        }
        if (exit_flag_) {
            break;
        }
        func_();
        {
            const std::lock_guard<std::mutex> lock{mutex_};
            work_flag_ = false;
        }
        condition_.notify_one();
    }
}

}  // namespace rally
