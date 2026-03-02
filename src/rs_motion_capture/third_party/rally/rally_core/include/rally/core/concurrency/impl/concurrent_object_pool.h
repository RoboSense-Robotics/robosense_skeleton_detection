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
#ifndef RALLY_CORE_CONCURRENCY_DETAILS_CONCURRENT_OBJECT_POOL_H
#define RALLY_CORE_CONCURRENCY_DETAILS_CONCURRENT_OBJECT_POOL_H

namespace rally {

template<typename T>
CCObjectPool<T>::CCObjectPool(uint32_t size) : capacity_(size) {
    capacity_ = capacity_ >= 1 ? capacity_ : 1;
    node_arena_ = static_cast<Node *>(checkedCalloc(capacity_, sizeof(Node)));
    for (int i = 0; i < static_cast<int>(capacity_ - 1); ++i) {
        node_arena_[i].next = node_arena_ + 1 + i;
    }
    node_arena_[capacity_ - 1].next = nullptr;
    free_head_.store({0, node_arena_}, std::memory_order_relaxed);
}

template<typename T>
std::shared_ptr <T> CCObjectPool<T>::getObject() {
    Head free_head;
    if (!findFreeHead(&free_head)) {
        return nullptr;
    }
    auto self = this->shared_from_this();
    return std::shared_ptr<T>(reinterpret_cast<T *>(free_head.node),
                              [self](T *object) { self->releaseObject(object); });
}

template<typename T>
void CCObjectPool<T>::releaseObject(T *object) {
    Head new_head;
    Node *node = reinterpret_cast<Node *>(object);
    Head old_head = free_head_.load(std::memory_order_acquire);
    do {
        node->next = old_head.node;
        new_head.node = node;
        new_head.count = old_head.count + 1;
    } while (!free_head_.compare_exchange_weak(old_head, new_head,
                                               std::memory_order_acq_rel,
                                               std::memory_order_acquire));
}

template<typename T>
bool CCObjectPool<T>::findFreeHead(Head *head) {
    Head new_head;
    Head old_head = free_head_.load(std::memory_order_acquire);
    do {
        if (old_head.node == nullptr) {
            return false;
        }
        new_head.node = old_head.node->next;
        new_head.count = old_head.count + 1;
    } while (!free_head_.compare_exchange_weak(old_head, new_head,
                                               std::memory_order_acq_rel,
                                               std::memory_order_acquire));
    *head = old_head;
    return true;
}

}  // namespace rally

#endif  // RALLY_CORE_CONCURRENCY_DETAILS_CONCURRENT_OBJECT_POOL_H
