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
#ifndef RALLY_CORE_CONCURRENCY_CONCURRENT_OBJECT_POOL_H
#define RALLY_CORE_CONCURRENCY_CONCURRENT_OBJECT_POOL_H

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>
#include "rally/utils/utils.h"
#include "rally/core/memory/memory.h"

namespace rally {

template<typename T>
class CCObjectPool : public std::enable_shared_from_this<CCObjectPool<T>> {
public:
    explicit CCObjectPool(uint32_t size);

    virtual ~CCObjectPool() { std::free(node_arena_); }

    template<typename... Args>
    void constructAll(Args &&... args) {
        for (uint32_t i = 0; i < capacity_; ++i) {
            new(node_arena_ + i) T(std::forward<Args>(args)...);
        }
    }

    template<typename... Args>
    std::shared_ptr<T> constructObject(Args &&... args) {
        Head free_head;
        if (!findFreeHead(&free_head)) {
            return nullptr;
        }
        auto self = this->shared_from_this();
        T *ptr = new(free_head.node) T(std::forward<Args>(args)...);
        return std::shared_ptr<T>(ptr, [self](T *object) {
            object->~T();
            self->releaseObject(object);
        });
    }

    std::shared_ptr<T> getObject();

    void releaseObject(T *object);

private:
    struct Node {
        T object;
        Node *next;
    };

    struct alignas(2 * sizeof(Node *)) Head {
        uintptr_t count;
        Node *node;
    };

    RALLY_DISALLOW_COPY_AND_ASSIGN(CCObjectPool)

    bool findFreeHead(Head *head);

    std::atomic<Head> free_head_;
    Node *node_arena_ = nullptr;
    uint32_t capacity_ = 0;
};

}  // namespace rally

#include "rally/core/concurrency/impl/concurrent_object_pool.h"

#endif  // RALLY_CORE_CONCURRENCY_CONCURRENT_OBJECT_POOL_H
