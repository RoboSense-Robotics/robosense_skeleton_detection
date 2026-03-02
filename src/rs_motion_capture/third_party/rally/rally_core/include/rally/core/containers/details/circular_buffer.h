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
#ifndef RALLY_CORE_CONTAINERS_DETAILS_CIRCULAR_BUFFER_H
#define RALLY_CORE_CONTAINERS_DETAILS_CIRCULAR_BUFFER_H

namespace rally {

template<typename T>
CircularBuffer<T>::CircularBuffer(size_t max_items) : max_items_(
max_items + 1) // one item is reserved as marker for full q
, v_(max_items_) {}

template<typename T>
CircularBuffer<T>::CircularBuffer(CircularBuffer &&other) noexcept {
    copy_moveable(std::move(other));
}

template<typename T>
void CircularBuffer<T>::push_back(const T &item) {
    if (max_items_ > 0) {
        v_[tail_] = std::move(item);
        tail_ = (tail_ + 1) % max_items_;

        if (tail_ == head_) {  // overrun last item if full
            head_ = (head_ + 1) % max_items_;
            ++overrun_counter_;
        }
    }
}

template<typename T>
size_t CircularBuffer<T>::size() const {
    if (tail_ >= head_) {
        return tail_ - head_;
    } else {
        return max_items_ - (head_ - tail_);
    }
}

template<typename T>
void CircularBuffer<T>::pop_front() {
    head_ = (head_ + 1) % max_items_;
}

template<typename T>
bool CircularBuffer<T>::empty() const {
    return tail_ == head_;
}

template<typename T>
    bool CircularBuffer<T>::full() const {
    // head is ahead of the tail by 1
    if (max_items_ > 0) {
        return ((tail_ + 1) % max_items_) == head_;
    }
    return false;
}

template<typename T>
size_t CircularBuffer<T>::overrun_counter() const {
    return overrun_counter_;
}

template<typename T>
void CircularBuffer<T>::copy_moveable(CircularBuffer &&other) noexcept {
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

}  // namespace rally

#endif  // RALLY_CORE_CONTAINERS_DETAILS_CIRCULAR_BUFFER_H
