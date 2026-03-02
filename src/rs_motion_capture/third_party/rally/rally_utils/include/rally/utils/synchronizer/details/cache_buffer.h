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
#ifndef RALLY_UTILS_SYNCHRONIZER_DETAILS_CACHE_BUFFER_H
#define RALLY_UTILS_SYNCHRONIZER_DETAILS_CACHE_BUFFER_H

#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace rally {

///@brief this code is crop from apollo

template <typename T>
class CacheBuffer {
 public:
  using value_type = T;
  using size_type = std::size_t;
  using FusionCallback = std::function<void(const T&)>;

  explicit CacheBuffer(uint64_t size) {
    capacity_ = size + 1;
    buffer_.resize(capacity_);
  }

  CacheBuffer(const CacheBuffer& rhs) {
    std::lock_guard<std::mutex> lg(rhs.mutex_);
    head_ = rhs.head_;
    tail_ = rhs.tail_;
    buffer_ = rhs.buffer_;
    capacity_ = rhs.capacity_;
  }

  T& operator[](const uint64_t& pos) { return buffer_[GetIndex(pos)]; }
  const T& at(const uint64_t& pos) const { return buffer_[GetIndex(pos)]; }

  uint64_t Head() const { return head_ + 1; }
  uint64_t Tail() const { return tail_; }
  uint64_t Size() const { return tail_ - head_; }

  const T& Front() const { return buffer_[GetIndex(head_ + 1)]; }
  const T& Back() const { return buffer_[GetIndex(tail_)]; }

  bool Empty() const { return tail_ == 0; }
  bool Full() const { return capacity_ - 1 == tail_ - head_; }
  uint64_t Capacity() const { return capacity_; }
  void Clear() { 
    head_ = 0;
    tail_ = 0;
    // buffer_.clear();
  }

  void Fill(const T& value) {
    if (Full()) {
      buffer_[GetIndex(head_)] = value;
      ++head_;
      ++tail_;
    } else {
      buffer_[GetIndex(tail_ + 1)] = value;
      ++tail_;
    }
  }

  std::mutex& Mutex() { return mutex_; }

 private:
  uint64_t GetIndex(const uint64_t& pos) const { return pos % capacity_; }

  uint64_t head_ = 0;
  uint64_t tail_ = 0;
  uint64_t capacity_ = 0;
  std::vector<T> buffer_;
  mutable std::mutex mutex_;
};

}  // namespace rally

#endif  // RALLY_UTILS_SYNCHRONIZER_DETAILS_CACHE_BUFFER_H
