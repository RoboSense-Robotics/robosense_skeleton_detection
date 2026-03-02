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
#ifndef RALLY_UTILS_SYNCHRONIZER_IMPL_EXACT_SYNCHRONIZER_H
#define RALLY_UTILS_SYNCHRONIZER_IMPL_EXACT_SYNCHRONIZER_H

#include "rally/utils/utils.h"

namespace rally {

template<typename AnySensor>
ExactSynchronizer<AnySensor>::ExactSynchronizer(const ExactSynOptions &options) {
    options_ = options;
    frame_id_set_.clear();
    for (size_t i = 0; i < options_.frame_id_vec.size(); ++i) {
        frame_id_set_.insert(options_.frame_id_vec[i]);
    }
    if (frame_id_set_.size() != options_.frame_id_vec.size()) {
        RTHROW(name() + ": setting with same frame_id!");
    }
}

template<typename AnySensor>
void ExactSynchronizer<AnySensor>::addData(const AnySensorPtr &msg_ptr) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (frame_id_set_.find(msg_ptr->getFrameID()) == frame_id_set_.end()) {
        RDEBUG << name() << ": add unknown frame_id " << msg_ptr->getFrameID();
        return;
    }
    auto &cur_map = queues_[msg_ptr->getTimestamp()];
    cur_map[msg_ptr->getFrameID()] = msg_ptr;

    // check
    if (cur_map.size() >= frame_id_set_.size()) {
        std::vector<AnySensorPtr> msg_ptr_vec;
        msg_ptr_vec.reserve(frame_id_set_.size());
        for (auto itr = cur_map.begin(); itr != cur_map.end(); ++itr) {
            msg_ptr_vec.push_back(itr->second);
        }
        for (auto cb : syn_cb_list_) {
            cb(msg_ptr_vec);
        }
        last_signal_time_ = msg_ptr->getTimestamp();
        queues_.erase(last_signal_time_);
        clearOldQueue();
    }

    if (options_.queue_size > 0) {
        while (queues_.size() > options_.queue_size) {
            queues_.erase(queues_.begin());
        }
    }
}

template<typename AnySensor>
void ExactSynchronizer<AnySensor>::regSynCallback(
const std::function<void(const std::vector<AnySensorPtr> &msg_ptr_vec)> &cb) {
    std::unique_lock<std::mutex> lock(reg_mutex_);
    syn_cb_list_.emplace_back(cb);
}

template<typename AnySensor>
void ExactSynchronizer<AnySensor>::clearOldQueue() {
    auto it = queues_.begin();
    auto end = queues_.end();

    for (; it != end;) {
        if (it->first <= last_signal_time_) {
            auto old = it;
            ++it;
            queues_.erase(old);
        } else {
            // the map is sorted by time, so we can ignore anything after this if this one's time is ok
            break;
        }
    }
}

}  // namespace rally

#endif  // RALLY_UTILS_SYNCHRONIZER_IMPL_EXACT_SYNCHRONIZER_H
