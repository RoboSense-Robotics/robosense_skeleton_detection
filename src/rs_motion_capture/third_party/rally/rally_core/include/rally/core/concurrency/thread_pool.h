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
#ifndef RALLY_CORE_CONCURRENCY_THREAD_POOL_H
#define RALLY_CORE_CONCURRENCY_THREAD_POOL_H

#include <functional>
#include <thread>
#include <atomic>
#include <vector>
#include <memory>
#include <exception>
#include <future>
#include <mutex>
#include <queue>

namespace rally {

namespace detail {
template<typename T>
class Queue {
public:
    bool push(T const &value) {
        std::unique_lock<std::mutex> lock(this->mutex);
        this->q.push(value);
        return true;
    }

    /// @brief deletes the retrieved element, do not use for non integral types
    bool pop(T &v) {
        std::unique_lock<std::mutex> lock(this->mutex);
        if (this->q.empty())
            return false;
        v = this->q.front();
        this->q.pop();
        return true;
    }

    bool empty() {
        std::unique_lock<std::mutex> lock(this->mutex);
        return this->q.empty();
    }

private:
    std::queue<T> q;
    std::mutex mutex;
};
}  // namespace detail

class ThreadPool {
public:

    ThreadPool() {
        this->init();
        this->resize(std::thread::hardware_concurrency());
    }

    ThreadPool(int32_t nThreads) {
        this->init();
        this->resize(nThreads);
    }

    /// @brief the destructor waits for all the functions in the queue to be finished
    ~ThreadPool() {
        this->stop(true);
    }

    /// @brief get the number of running threads in the pool
    size_t size() { return static_cast<size_t>(this->threads.size()); }

    std::thread &get_thread(int32_t i) { return *this->threads[i]; }

    /// @brief change the number of threads in the pool
    ///        should be called from one thread, otherwise be careful to not interleave, also with this->stop()
    ///        nThreads must be >= 0
    void resize(int32_t nThreads) noexcept;

    /// @brief empty the queue
    void clear_queue() noexcept;

    /// @brief pops a functional wrapper to the original function
    std::function<void(int32_t)> pop() noexcept;

    /// @brief wait for all computing threads to finish and stop all threads
    ///        may be called asynchronously to not pause the calling thread while waiting
    ///        if isWait == true, all the functions in the queue are run, otherwise the
    ///        queue is cleared without running the functions
    void stop(bool isWait = false) noexcept;

    template<typename F, typename... Rest>
    auto push(F &&f, Rest &&... rest) -> std::future<decltype(f(0, rest...))> {
        auto pck = std::make_shared<std::packaged_task<decltype(f(0, rest...))(int32_t)>>(
        std::bind(std::forward<F>(f), std::placeholders::_1, std::forward<Rest>(rest)...)
        );
        auto _f = new std::function<void(int32_t id)>([pck](int32_t id) {
            (*pck)(id);
        });
        this->q.push(_f);
        std::unique_lock<std::mutex> lock(this->mutex);
        this->cv.notify_one();
        return pck->get_future();
    }

    /// @brief run the user's function that excepts argument int - id of the running thread. returned value is templatized
    ///        operator returns std::future, where the user can get the result and rethrow the catched exceptins
    template<typename F>
    auto push(F &&f) -> std::future<decltype(f(0))> {
        auto pck = std::make_shared<std::packaged_task<decltype(f(0))(int32_t)>>(std::forward<F>(f));
        auto _f = new std::function<void(int32_t id)>([pck](int32_t id) {
            (*pck)(id);
        });
        this->q.push(_f);
        std::unique_lock<std::mutex> lock(this->mutex);
        this->cv.notify_one();
        return pck->get_future();
    }

private:

    // deleted
    ThreadPool(const ThreadPool &);// = delete;
    ThreadPool(ThreadPool &&);// = delete;
    ThreadPool &operator=(const ThreadPool &);// = delete;
    ThreadPool &operator=(ThreadPool &&);// = delete;

    void set_thread(int32_t i) noexcept;

    void init() {
        this->nWaiting = 0;
        this->isStop = false;
        this->isDone = false;
    }

    std::vector<std::unique_ptr<std::thread>> threads;
    std::vector<std::shared_ptr<std::atomic<bool>>> flags;
    detail::Queue<std::function<void(int32_t id)> *> q;
    std::atomic<bool> isDone;
    std::atomic<bool> isStop;
    std::atomic<int32_t> nWaiting;  // how many threads are waiting

    std::mutex mutex;
    std::condition_variable cv;
};

}  // namespace rally

#endif  // RALLY_CORE_CONCURRENCY_THREAD_POOL_H
