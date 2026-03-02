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
#ifndef RALLY_CORE_CONCURRENCY_DETAILS_BOUNDED_QUEUE_H
#define RALLY_CORE_CONCURRENCY_DETAILS_BOUNDED_QUEUE_H

namespace rally {

template<typename T>
BoundedQueue<T>::~BoundedQueue() {
    if (wait_strategy_) {
        breakAllWait();
    }
    if (pool_) {
        for (uint64_t i = 0; i < pool_size_; ++i) {
            pool_[i].~T();
        }
        std::free(pool_);
    }
}

template<typename T>
bool BoundedQueue<T>::init(uint64_t size) {
    return init(size, new SleepWaitStrategy());
}

template<typename T>
bool BoundedQueue<T>::init(uint64_t size, WaitStrategy *strategy) {
    // Head and tail each occupy a space
    pool_size_ = size + 2;
    pool_ = reinterpret_cast<T *>(std::calloc(pool_size_, sizeof(T)));
    if (pool_ == nullptr) {
        return false;
    }
    for (uint64_t i = 0; i < pool_size_; ++i) {
        new(&(pool_[i])) T();
    }
    wait_strategy_.reset(strategy);
    return true;
}

template<typename T>
bool BoundedQueue<T>::enqueue(const T &element) {
    uint64_t new_tail = 0;
    uint64_t old_commit = 0;
    uint64_t old_tail = tail_.load(std::memory_order_acquire);
    do {
        new_tail = old_tail + 1;
        if (getIndex(new_tail) == getIndex(head_.load(std::memory_order_acquire))) {
            return false;
        }
    } while (!tail_.compare_exchange_weak(old_tail, new_tail,
                                          std::memory_order_acq_rel,
                                          std::memory_order_relaxed));
    pool_[getIndex(old_tail)] = element;
    do {
        old_commit = old_tail;
    } while (!commit_.compare_exchange_weak(
    old_commit, new_tail, std::memory_order_acq_rel,
    std::memory_order_relaxed));
    wait_strategy_->notifyOne();
    return true;
}

template<typename T>
bool BoundedQueue<T>::enqueue(T &&element) {
    uint64_t new_tail = 0;
    uint64_t old_commit = 0;
    uint64_t old_tail = tail_.load(std::memory_order_acquire);
    do {
        new_tail = old_tail + 1;
        if (getIndex(new_tail) == getIndex(head_.load(std::memory_order_acquire))) {
            return false;
        }
    } while (!tail_.compare_exchange_weak(old_tail, new_tail,
                                          std::memory_order_acq_rel,
                                          std::memory_order_relaxed));
    pool_[getIndex(old_tail)] = std::move(element);
    do {
        old_commit = old_tail;
    } while (!commit_.compare_exchange_weak(
    old_commit, new_tail, std::memory_order_acq_rel,
    std::memory_order_relaxed));
    wait_strategy_->notifyOne();
    return true;
}

template<typename T>
bool BoundedQueue<T>::waitEnqueue(const T &element) {
    while (!break_all_wait_) {
        if (enqueue(element)) {
            return true;
        }
        if (wait_strategy_->emptyWait()) {
            continue;
        }
        // wait timeout
        break;
    }

    return false;
}

template<typename T>
bool BoundedQueue<T>::dequeue(T *element) {
    uint64_t new_head = 0;
    uint64_t old_head = head_.load(std::memory_order_acquire);
    do {
        new_head = old_head + 1;
        if (new_head == commit_.load(std::memory_order_acquire)) {
            return false;
        }
        *element = pool_[getIndex(new_head)];
    } while (!head_.compare_exchange_weak(old_head, new_head,
                                          std::memory_order_acq_rel,
                                          std::memory_order_relaxed));
    return true;
}

template<typename T>
bool BoundedQueue<T>::waitDequeue(T *element) {
    while (!break_all_wait_) {
        if (dequeue(element)) {
            return true;
        }
        if (wait_strategy_->emptyWait()) {
            continue;
        }
        // wait timeout
        break;
    }

    return false;
}

template<typename T>
uint64_t BoundedQueue<T>::size() {
    return tail_ - head_ - 1;
}

template<typename T>
bool BoundedQueue<T>::empty() {
    return size() == 0;
}

template<typename T>
void BoundedQueue<T>::breakAllWait() {
    break_all_wait_ = true;
    wait_strategy_->breakAllWait();
}

}  // namespace rally

#endif  // RALLY_CORE_CONCURRENCY_DETAILS_BOUNDED_QUEUE_H
