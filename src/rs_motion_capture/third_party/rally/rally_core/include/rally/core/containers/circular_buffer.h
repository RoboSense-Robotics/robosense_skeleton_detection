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
#ifndef RALLY_CORE_CONTAINERS_CIRCULAR_BUFFER_H
#define RALLY_CORE_CONTAINERS_CIRCULAR_BUFFER_H

#include <vector>
#include <cassert>
#include <string>

namespace rally {

template<typename T>
class CircularBuffer {
public:
    using value_type = T;

    /// @brief empty ctor - create a disabled queue with no elements allocated at all
    CircularBuffer() = default;

    explicit CircularBuffer(size_t max_items);

    CircularBuffer(const CircularBuffer &) = default;

    CircularBuffer &operator=(const CircularBuffer &) = default;

    /// @brief move cannot be default,
    ///        since we need to reset head_, tail_, etc to zero in the moved object
    CircularBuffer(CircularBuffer &&other) noexcept;

    CircularBuffer &operator=(CircularBuffer &&other) noexcept {
        copy_moveable(std::move(other));
        return *this;
    }

    /// @brief push back, overrun (oldest) item if no room left
    void push_back(const T &item);

    /// @brief Return reference to the front item.
    ///        If there are no elements in the container, the behavior is undefined.
    const T &front() const {
        return v_[head_];
    }

    T &front() {
        return v_[head_];
    }

    /// @brief Return number of elements actually stored
    size_t size() const;

    /// @brief Return const reference to item by index.
    ///        If index is out of range 0…size()-1, the behavior is undefined.
    const T &at(size_t i) const {
        assert(i < size());
        return v_[(head_ + i) % max_items_];
    }

    /// @brief Pop item from front.
    ///        If there are no elements in the container, the behavior is undefined.
    void pop_front();

    bool empty() const;

    bool full() const;

    size_t overrun_counter() const;

private:
    /// @brief copy from other&& and reset it to disabled state
    void copy_moveable(CircularBuffer &&other) noexcept;

    size_t max_items_ = 0;
    typename std::vector<T>::size_type head_ = 0;
    typename std::vector<T>::size_type tail_ = 0;
    size_t overrun_counter_ = 0;
    std::vector<T> v_;
};

}  // namespace rally

#include "rally/core/containers/details/circular_buffer.h"

#endif  // RALLY_CORE_CONTAINERS_CIRCULAR_BUFFER_H
