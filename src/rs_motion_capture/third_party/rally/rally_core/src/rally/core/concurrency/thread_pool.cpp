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

#include "rally/core/concurrency/thread_pool.h"

namespace rally {

void ThreadPool::resize(int32_t nThreads) noexcept {
    const bool tmp_stop{this->isStop.load()};
    const bool tmp_done{this->isDone.load()};
    if (!tmp_stop && !tmp_done) {
        const int32_t oldNThreads{static_cast<int32_t>(this->threads.size())};
        if (oldNThreads <= nThreads) {  // if the number of threads is increased
            this->threads.resize(static_cast<size_t>(nThreads));
            this->flags.resize(static_cast<size_t>(nThreads));

            for (int32_t i{oldNThreads}; i < nThreads; ++i) {
                this->flags[i] = std::make_shared<std::atomic<bool>>(false);
                this->set_thread(i);
            }
        } else {  // the number of threads is decreased
            for (int32_t i{oldNThreads - 1}; i >= nThreads; --i) {
                *this->flags[i] = true;  // this thread will finish
                this->threads[i]->detach();
            }
            {
                // stop the detached threads that were waiting
                const std::unique_lock<std::mutex> lock{this->mutex};
                this->cv.notify_all();
            }
            this->threads.resize(static_cast<size_t>(nThreads));  // safe to delete because the threads are detached
            this->flags.resize(
            static_cast<size_t>(nThreads));  // safe to delete because the threads have copies of shared_ptr of the flags, not originals
        }
    }
}

void ThreadPool::clear_queue() noexcept {
    std::function<void(int32_t id)> *_f;
    while (this->q.pop(_f)) {
        delete _f; // empty the queue
    }
}

std::function<void(int32_t)> ThreadPool::pop() noexcept {
    std::function<void(int32_t id)> *_f{nullptr};
    static_cast<void>(this->q.pop(_f));
    std::function<void(int32_t)> f;
    if (_f) {
        f = *_f;
    }
    return f;
}

void ThreadPool::stop(bool isWait) noexcept {
    if (!isWait) {
        const bool tmp_stop{this->isStop.load()};
        if (tmp_stop) {
            return;
        }
        this->isStop = true;
        for (size_t i{0}; i < this->size(); ++i) {
            *this->flags[i] = true;  // command the threads to stop
        }
        this->clear_queue();  // empty the queue
    } else {
        const bool tmp_stop{this->isStop.load()};
        const bool tmp_done{this->isDone.load()};
        if (tmp_done || tmp_stop) {
            return;
        }
        this->isDone = true;  // give the waiting threads a command to finish
    }
    {
        const std::unique_lock<std::mutex> lock{this->mutex};
        this->cv.notify_all();  // stop all waiting threads
    }
    for (size_t i{0}; i < this->threads.size(); ++i) {  // wait for the computing threads to finish
        if (this->threads[i]->joinable()) {
            this->threads[i]->join();
        }
    }
    // if there were no threads in the pool but some functors in the queue, the functors are not deleted by the threads
    // therefore delete them here
    this->clear_queue();
    this->threads.clear();
    this->flags.clear();
}

void ThreadPool::set_thread(int32_t i) noexcept {
    auto f = [this, i]() noexcept -> void {
        const auto &flag = this->flags[static_cast<size_t>(i)];
        std::function<void(int32_t id)> *_f;
        bool isPop{this->q.pop(_f)};
        while (true) {
            while (isPop) {  // if there is anything in the queue
                (*_f)(i);
                const auto tmp_flag = flag->load();
                if (tmp_flag) {
                    return;  // the thread is wanted to stop, return even if the queue is not empty yet
                } else {
                    isPop = this->q.pop(_f);
                }
            }
            // the queue is empty here, wait for the next command
            std::unique_lock<std::mutex> lock{this->mutex};
            ++this->nWaiting;
            this->cv.wait(lock, [this, &_f, &isPop, flag]() noexcept {
                isPop = this->q.pop(_f);
                const bool tmp_flag{flag->load()};
                const bool tmp_done{this->isDone.load()};
                return isPop || tmp_done || tmp_flag;
            });
            --this->nWaiting;
            if (!isPop) {
                return;  // if the queue is empty and this->isDone == true or *flag then return
            }
        }
    };
    this->threads[i] = std::make_unique<std::thread>(f);
}

}  // namespace rally
