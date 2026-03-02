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
#ifndef HYPER_VISION_PROMETHEUS_BASE_BOUNDED_QUEUE_H
#define HYPER_VISION_PROMETHEUS_BASE_BOUNDED_QUEUE_H

#include <unistd.h>
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <utility>
#include "rally/utils/details/macros.h"
#include "rally/core/concurrency/wait_strategy.h"

namespace rally {

template<typename T>
class BoundedQueue {
public:
    BoundedQueue() {}

    ~BoundedQueue();

    bool init(uint64_t size);

    bool init(uint64_t size, WaitStrategy *strategy);

    bool enqueue(const T &element);

    bool enqueue(T &&element);

    bool waitEnqueue(const T &element);

    bool dequeue(T *element);

    bool waitDequeue(T *element);

    uint64_t size();

    bool empty();

    void breakAllWait();

private:
    RALLY_DISALLOW_COPY_AND_ASSIGN(BoundedQueue)

    uint64_t getIndex(uint64_t num) {
        return num - (num / pool_size_) * pool_size_;  // faster than %
    }

    alignas(RALLY_CACHELINE_SIZE) std::atomic<uint64_t> head_ = {0};
    alignas(RALLY_CACHELINE_SIZE) std::atomic<uint64_t> tail_ = {1};
    alignas(RALLY_CACHELINE_SIZE) std::atomic<uint64_t> commit_ = {1};
    uint64_t pool_size_ = 0;
    T *pool_ = nullptr;
    std::unique_ptr<WaitStrategy> wait_strategy_ = nullptr;
    volatile bool break_all_wait_ = false;
};

}  // namespace rally

#include "rally/core/concurrency/impl/bounded_queue.h"

#endif  // HYPER_VISION_PROMETHEUS_BASE_BOUNDED_QUEUE_H
