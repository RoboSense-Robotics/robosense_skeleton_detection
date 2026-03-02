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
#ifndef RALLY_UTILS_LOGGER_DETAIL_CIRCULAR_Q_H
#define RALLY_UTILS_LOGGER_DETAIL_CIRCULAR_Q_H

#include <vector>
#include <cassert>
#include <string>

namespace rally {
namespace log {

template<typename T>
class CircularQ {
public:
    using value_type = T;

    // empty ctor - create a disabled queue with no elements allocated at all
    CircularQ() = default;

    explicit CircularQ(size_t max_items) : max_items_(max_items + 1) // one item is reserved as marker for full q
    , v_(max_items_)
    {}

    CircularQ(const CircularQ &) = default;
    CircularQ &operator=(const CircularQ &) = default;

    // move cannot be default,
    // since we need to reset head_, tail_, etc to zero in the moved object
    CircularQ(CircularQ &&other) noexcept {
        copy_moveable(std::move(other));
    }

    CircularQ &operator=(CircularQ &&other) noexcept {
        copy_moveable(std::move(other));
        return *this;
    }

    // push back, overrun (oldest) item if no room left
    void push_back(T &&item) {
        if (max_items_ > 0) {
            v_[tail_] = std::move(item);
            tail_ = (tail_ + 1) % max_items_;

            if (tail_ == head_) {  // overrun last item if full
                head_ = (head_ + 1) % max_items_;
                ++overrun_counter_;
            }
        }
    }

    // Return reference to the front item.
    // If there are no elements in the container, the behavior is undefined.
    const T &front() const {
        return v_[head_];
    }

    T &front() {
        return v_[head_];
    }

    // Return number of elements actually stored
    size_t size() const {
        if (tail_ >= head_) {
            return tail_ - head_;
        } else {
            return max_items_ - (head_ - tail_);
        }
    }

    // Return const reference to item by index.
    // If index is out of range 0…size()-1, the behavior is undefined.
    const T &at(size_t i) const {
        assert(i < size());
        return v_[(head_ + i) % max_items_];
    }

    // Pop item from front.
    // If there are no elements in the container, the behavior is undefined.
    void pop_front() {
        head_ = (head_ + 1) % max_items_;
    }

    bool empty() const {
        return tail_ == head_;
    }

    bool full() const {
        // head is ahead of the tail by 1
        if (max_items_ > 0) {
            return ((tail_ + 1) % max_items_) == head_;
        }
        return false;
    }

    size_t overrun_counter() const {
        return overrun_counter_;
    }

private:
    // copy from other&& and reset it to disabled state
    void copy_moveable(CircularQ &&other) noexcept {
        max_items_ = other.max_items_;
        head_ = other.head_;
        tail_ = other.tail_;
        overrun_counter_ = other.overrun_counter_;
        v_ = std::move(other.v_);

        // put &&other in disabled, but valid state
        other.max_items_ = 0;
        other.head_ = other.tail_ = 0;
        other.overrun_counter_ = 0;
    }

    size_t max_items_ = 0;
    typename std::vector<T>::size_type head_ = 0;
    typename std::vector<T>::size_type tail_ = 0;
    size_t overrun_counter_ = 0;
    std::vector<T> v_;
};

}  // namespace log
}  // namespace rally

#endif  // RALLY_UTILS_LOGGER_DETAIL_CIRCULAR_Q_H
