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
#ifndef RALLY_UTILS_SYNCHRONIZER_MAIN_SENSOR_EPSILON_SYNCHRONIZER_H
#define RALLY_UTILS_SYNCHRONIZER_MAIN_SENSOR_EPSILON_SYNCHRONIZER_H

#include <condition_variable>
#include <deque>
#include <functional>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "rally/utils/synchronizer/details/details.h"
#include "rally/utils/utils.h"
#include "rally/utils/synchronizer/details/cache_buffer.h"

namespace rally {

struct MainSensorSynOptions {
  std::vector<std::string> frame_id_vec;
  uint64_t epsilon = 200000000U;  // 单位是ns，epsilon默认为10ms~
  std::string main_sensor_frame_id;
  std::vector<uint64_t> queue_size_vec;
};

template <typename AnySensor>
class MainSensorSynchronizer {
  RALLY_STATIC_ASSERT(details::has_getTimestamp<AnySensor>::value
                          &&details::has_getFrameID<AnySensor>::value);

 public:
  using Ptr = std::unique_ptr<MainSensorSynchronizer>;
  using AnySensorPtr = std::shared_ptr<AnySensor>;

  explicit MainSensorSynchronizer(
      const MainSensorSynOptions &options = MainSensorSynOptions()) {
    options_ = options;
    sensor_msg_map_.clear();
    for (size_t i = 0; i < options_.frame_id_vec.size(); ++i) {
      RENSURE(!options_.frame_id_vec[i].empty());

      sensor_msg_map_.insert(
          {options_.frame_id_vec[i],
           CacheBuffer<AnySensorPtr>(options_.queue_size_vec[i])});
      //        sensor_msg_map_.insert({options_.frame_id_vec[i],
      //        CacheBuffer<AnySensorPtr>(options_.queue_size[i])});
      deque_len_ += options_.queue_size_vec[i];
    }
    deque_len_ *= 3;

    RENSURE(deque_len_ > 0);
    RENSURE(sensor_msg_map_.size() == options_.frame_id_vec.size());
    RENSURE(!options_.main_sensor_frame_id.empty());
    RENSURE(sensor_msg_map_.find(options_.main_sensor_frame_id) !=
            sensor_msg_map_.end());
  }

  void addData(const AnySensorPtr &msg_ptr) {
    std::unique_lock<std::mutex> lock(mutex_);
    while (msg_deque_.size() >= deque_len_){
      msg_deque_.pop_front();
    }
    
    msg_deque_.push_back(msg_ptr);
    condition_.notify_all();
  }

  void start() {
    run_flag_ = true;
    if (thread_ptr_ == nullptr) {
      thread_ptr_ =
          std::make_unique<std::thread>(&MainSensorSynchronizer::core, this);
    }
  }

  void stop() {
    if (thread_ptr_ != nullptr) {
      run_flag_ = false;
      condition_.notify_all();
      if (thread_ptr_->joinable()) {
        thread_ptr_->join();
      }
    }
  }

  ~MainSensorSynchronizer() { stop(); }

  const std::unique_ptr<std::thread> &getTd() const { return thread_ptr_; }

  void regSynCallback(
      const std::function<void(const std::vector<AnySensorPtr> &msg_ptr_vec)>
          &cb) {
    std::unique_lock<std::mutex> lock(mutex_);
    syn_cb_list_.emplace_back(cb);
  }

 private:
  const std::string name() const { return "MainSensorSynchronizer"; }

  void core() {
    while (run_flag_ || !msg_deque_.empty()) {
      std::deque<AnySensorPtr> data;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (msg_deque_.empty()) {
          condition_.wait(lock);
          if (!run_flag_) {
            continue;
          }
        }
        data = msg_deque_;
        msg_deque_.clear();
      }

      while (!data.empty()) {
        std::unique_lock<std::mutex> lock(mutex_);
        auto msg_ptr = data.front();
        data.pop_front();
        if (sensor_msg_map_.find(msg_ptr->getFrameID()) ==
            sensor_msg_map_.end()) {
          RWARN << name() << ": receive unknown frame_id!"
                << msg_ptr->getFrameID();
          continue;
        }

        auto &deque = sensor_msg_map_.at(msg_ptr->getFrameID());
        // step1. 检查消息队列的时间序是否异常，异常则给出警告，但是不影响运行
        if (!deque.Empty()) {
          const auto &latest_data = deque.Back();
          if (latest_data->getTimestamp() >= msg_ptr->getTimestamp()) {
            RWARN << name() << ": receive msg with wrong timestamp sequence!";
          }
        }
        if (msg_ptr->getFrameID() != options_.main_sensor_frame_id) {
          deque.Fill(msg_ptr);
        } else {
          std::vector<AnySensorPtr> sensor_ptr_vec = {msg_ptr};
          for (auto itr = sensor_msg_map_.begin(); itr != sensor_msg_map_.end();
               ++itr) {
            if (itr->second.Empty() || itr->first == options_.main_sensor_frame_id) {
              continue;
            }

            uint64_t cur_epsilon;
            uint64_t min_epsilon = std::numeric_limits<uint64_t>::max();
            int target_idx = -1;
            for (int i = itr->second.Head(); i <= itr->second.Tail(); ++i) {
              auto &sensor_msg_ptr = itr->second[i];
              if (msg_ptr->getTimestamp() > sensor_msg_ptr->getTimestamp()) {
                cur_epsilon =
                    msg_ptr->getTimestamp() - sensor_msg_ptr->getTimestamp();
              } else {
                cur_epsilon =
                    sensor_msg_ptr->getTimestamp() - msg_ptr->getTimestamp();
              }
              target_idx = cur_epsilon < min_epsilon ? i : target_idx;
              min_epsilon =
                  cur_epsilon < min_epsilon ? cur_epsilon : min_epsilon;
            }

            if (target_idx < itr->second.Head() ||
                target_idx > itr->second.Tail() ||
                min_epsilon > options_.epsilon) {
              continue;
            }
            auto &target_msg = itr->second[target_idx];
            sensor_ptr_vec.emplace_back(target_msg);
          }
          for (size_t i = 0; i < syn_cb_list_.size(); ++i) {
            syn_cb_list_[i](sensor_ptr_vec);
          }
        }
      }
    }
  }

  MainSensorSynOptions options_;
  bool is_map_full_{false};
  std::unique_ptr<std::thread> thread_ptr_;

  std::map<std::string, CacheBuffer<AnySensorPtr>> sensor_msg_map_;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool run_flag_ = false;
  std::deque<AnySensorPtr> msg_deque_;

  uint64_t deque_len_ = 0;

  std::vector<std::function<void(const std::vector<AnySensorPtr> &msg_ptr_vec)>>
      syn_cb_list_;
};

}  // namespace rally

#endif  // RALLY_UTILS_SYNCHRONIZER_MAIN_SENSOR_EPSILON_SYNCHRONIZER_H
