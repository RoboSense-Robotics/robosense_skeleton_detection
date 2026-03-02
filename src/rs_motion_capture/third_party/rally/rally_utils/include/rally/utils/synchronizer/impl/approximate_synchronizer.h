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
#ifndef RALLY_UTILS_SYNCHRONIZER_IMPL_APPROXIMATE_SYNCHRONIZER_H
#define RALLY_UTILS_SYNCHRONIZER_IMPL_APPROXIMATE_SYNCHRONIZER_H

#include "rally/utils/utils.h"

namespace rally {

template<typename AnySensor>
ApproximateSynchronizer<AnySensor>::ApproximateSynchronizer(const ApproximateSynOptions &options) {
    options_ = options;
    // queue_size_ = options_.queue_size;
    age_penalty_ = options_.age_penalty;
    msg_deque_.clear();
    sensor_msg_map.clear();
    syn_cb_list_.clear();
    num_non_empty_deques_ = 0;
    pivot_ = NO_PIVOT;

    for (size_t i = 0; i < options_.frame_id_vec.size(); ++i) {
        const auto &frame_id = options_.frame_id_vec[i];
        std::deque<AnySensorPtr> tmp_msg_deque;
        std::vector<AnySensorPtr> tmp_msg_vector;
        sensor_msg_map[frame_id] = tmp_msg_deque;
        sensor_msg_past[frame_id] = tmp_msg_vector;
        has_dropped_messages_[frame_id] = false;
        inter_message_lower_bounds_[frame_id] = options_.inter_message_lower_bound[i] * 1000000UL;
        queue_size_[frame_id] = options_.queue_size[i];
    }
}

template<typename AnySensor>
void ApproximateSynchronizer<AnySensor>::addData(const AnySensorPtr &msg_ptr) {
    std::unique_lock<std::mutex> lock(mutex_);
    msg_deque_.push_back(msg_ptr);
    condition_.notify_all();
}

template<typename AnySensor>
void ApproximateSynchronizer<AnySensor>::start() {
    run_flag_ = true;
    if (thread_ptr_ == nullptr) {
        thread_ptr_ = std::make_unique<std::thread>(&ApproximateSynchronizer::core, this);
    }
}

template<typename AnySensor>
void ApproximateSynchronizer<AnySensor>::stop() {
    if (thread_ptr_ != nullptr) {
        run_flag_ = false;
        condition_.notify_all();
        if (thread_ptr_->joinable()) {
            thread_ptr_->join();
        }
    }
}

template<typename AnySensor>
ApproximateSynchronizer<AnySensor>::~ApproximateSynchronizer() {
    stop();
}

template<typename AnySensor>
const std::unique_ptr<std::thread> &ApproximateSynchronizer<AnySensor>::getTd() const {
    return thread_ptr_;
}

template<typename AnySensor>
void ApproximateSynchronizer<AnySensor>::regSynCallback(
const std::function<void(const std::vector<AnySensorPtr> &msg_ptr_vec)> &cb) {
    std::unique_lock<std::mutex> lock(mutex_);
    syn_cb_list_.emplace_back(cb);
}

template<typename AnySensor>
void ApproximateSynchronizer<AnySensor>::core() {
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
            auto msg_ptr = data.front();
            data.pop_front();
            if (sensor_msg_map.find(msg_ptr->getFrameID()) == sensor_msg_map.end()) {
                RWARN << name() << ": receive unknown frame_id!" << msg_ptr->getFrameID();
                continue;
            }
            auto &deque = sensor_msg_map.at(msg_ptr->getFrameID());
            deque.push_back(msg_ptr);

            // 1. 正常情况,所有队列都收到数据了，则进行处理，走process处理流程
            if (deque.size() == (size_t) 1) {
                ++num_non_empty_deques_;
                if (num_non_empty_deques_ == (uint32_t) sensor_msg_map.size()) {
                    process();
                }
            } else {
                // 检查当前这个frame_id的消息序列,是否出现时间非顺序的情况,仅给出warning
                checkInterMessageBound(msg_ptr->getFrameID());
            }

            // 2. 检查新来的数据所在的队列大小是否已经超过限制
            auto &past = sensor_msg_past.at(msg_ptr->getFrameID());
            if (past.size() + deque.size() > queue_size_.at(msg_ptr->getFrameID())) {
                // 超过则重置
                num_non_empty_deques_ = 0;
                recover();

                // 这是异常情况，几乎不可能发生
                if (deque.empty()) {
                    RTHROW(name() + ": deque is empty!");
                }
                // 超过限制的那个队列，抛弃旧有的数据，并且使用has_dropped_messages_进行标记
                deque.pop_front();
                has_dropped_messages_.at(msg_ptr->getFrameID()) = true;
                // 清除临时标记，并走process处理流程
                if (pivot_ != NO_PIVOT) {
                    candidate_.clear();
                    pivot_ = NO_PIVOT;
                    process();
                }
            }
        }
    }
}

template<typename AnySensor>
void ApproximateSynchronizer<AnySensor>::process() {
    // 走处理流程的必须条件
    while (num_non_empty_deques_ == static_cast<uint32_t>(sensor_msg_map.size())) {
        uint64_t end_time, start_time;
        std::string end_frame_id, start_frame_id;
        // 找到所有队列中，最前面的那个数据，并且对比找到时间最早和最晚的数据
        getCandidateStart(start_frame_id, start_time);
        getCandidateEnd(end_frame_id, end_time);

        // 这一句实际没啥用，主要就是对于最晚的数据，如果也是刚drop过的数据，说明同步candidate已经出现了滞后，则需要重新找更优的candidate
        for (auto itr = has_dropped_messages_.begin(); itr != has_dropped_messages_.end(); ++itr) {
            if (itr->first != end_frame_id) {
                itr->second = false;
            }
        }
        // 其实是个标记，用于标记是否已经找到了锚点，所谓锚点，就是已经找到了一组candidate，而锚点则是那个最晚的时间frameid，合理
        if (pivot_ == NO_PIVOT) {
            // 如果同步序列的时间差大于了预设值,则干掉最早的那个目标,重新寻找同步序列
            if (end_time - start_time > options_.max_interval_duration) {
                dequeDeleteFront(start_frame_id);
                continue;
            }
            // 如果end_frame_id所在的序列是一个刚刚才drop了的序列,那就说明这不是一个好的pivot,就需要删掉start_frame_id.重新找同步序列
            if (has_dropped_messages_.at(end_frame_id)) {
                dequeDeleteFront(start_frame_id);
                continue;
            }
            // 找到同步序列
            makeCandidate();
            // 找到锚点，之后则是检查是否队列中有比当前candidate更优的candidate
            candidate_start_ = start_time;
            candidate_end_ = end_time;
            pivot_ = end_frame_id;
            pivot_time_ = end_time;
            // 将最早的那个队列中的数据放到past队列中，用来干啥？
            dequeMoveFrontToPast(start_frame_id);
        } else {
            if ((end_time - candidate_end_) * (1 + age_penalty_) >= (start_time - candidate_start_)) {
                dequeMoveFrontToPast(start_frame_id);
            } else {
                makeCandidate();
                candidate_start_ = start_time;
                candidate_end_ = end_time;
                dequeMoveFrontToPast(start_frame_id);
            }
        }

        // 还没有找到锚点就走到这一步，那就是异常，证明代码逻辑有问题
        if (pivot_ == NO_PIVOT) {
            RTHROW(name() + ": pivot euqal to NO_PIVOT!");
        }

        // 最晚时间的frameid和锚点一样了，说明这个队列是最优的了。释放
        if (start_frame_id == pivot_) {
            publishCandidate();
        } else if ((end_time - candidate_end_) * (1 + age_penalty_) >= (pivot_time_ - candidate_start_)) {
            publishCandidate();
        } else if (num_non_empty_deques_ < static_cast<uint32_t>(sensor_msg_map.size())) {
            uint32_t num_non_empty_deques_before_virtual_search = num_non_empty_deques_;

            std::map<std::string, int> num_virtual_moves_map;
            for (auto itr = sensor_msg_map.begin(); itr != sensor_msg_map.end(); ++itr) {
                num_virtual_moves_map[itr->first] = 0;
            }
            while (1) {
                uint64_t end_time, start_time;
                std::string end_frame_id, start_frame_id;
                getVirtualCandidateEnd(end_frame_id, end_time);
                getVirtualCandidateStart(start_frame_id, start_time);
                if ((end_time - candidate_end_) * (1 + age_penalty_) >= (pivot_time_ - candidate_start_)) {
                    publishCandidate();
                    break;
                }
                if ((end_time - candidate_end_) * (1 + age_penalty_) < (start_time - candidate_start_)) {
                    num_non_empty_deques_ = 0;
                    for (auto itr = num_virtual_moves_map.begin(); itr != num_virtual_moves_map.end(); ++itr) {
                        recover(itr->first, itr->second);
                    }
                    if (num_non_empty_deques_before_virtual_search != num_non_empty_deques_) {
                        RTHROW(name() + ": num_non_empty_deques_before_virtual_search "
                                        "not equal to num_non_empty_deques!");
                    }
                    break;
                }
                if (start_frame_id == pivot_) {
                    RTHROW(name() + ": start_frame_id equal to pivot!");
                }
                if (start_time >= pivot_time_) {
                    RTHROW(name() + ": start_time larger than pivot_time!");
                }
                dequeMoveFrontToPast(start_frame_id);
                num_virtual_moves_map.at(start_frame_id)++;
            }
        }
    }
}

template<typename AnySensor>
void ApproximateSynchronizer<AnySensor>::getCandidateEnd(std::string &end_frame_id, uint64_t &end_time) {
    return getCandidateBoundary(end_frame_id, end_time, true);
}

template<typename AnySensor>
void ApproximateSynchronizer<AnySensor>::getCandidateStart(std::string &start_frame_id, uint64_t &start_time) {
    return getCandidateBoundary(start_frame_id, start_time, false);
}

template<typename AnySensor>
void ApproximateSynchronizer<AnySensor>::getCandidateBoundary(std::string &frame_id, uint64_t &time, bool end) {
    auto first_itr = sensor_msg_map.begin();
    time = first_itr->second.front()->getTimestamp();
    frame_id = first_itr->second.front()->getFrameID();

    for (auto itr = sensor_msg_map.begin(); itr != sensor_msg_map.end(); ++itr) {
        if ((itr->second.front()->getTimestamp() < time) ^ end) {
            time = itr->second.front()->getTimestamp();
            frame_id = itr->second.front()->getFrameID();
        }
    }
}

template<typename AnySensor>
void ApproximateSynchronizer<AnySensor>::dequeDeleteFront(const std::string &frame_id) {
    auto &deque = sensor_msg_map.at(frame_id);
    if (deque.empty()) {
        RTHROW(name() + ": deque is empty!");
    }
    deque.pop_front();
    if (deque.empty()) {
        --num_non_empty_deques_;
    }
}

template<typename AnySensor>
void ApproximateSynchronizer<AnySensor>::makeCandidate() {
    candidate_.clear();
    for (auto itr = sensor_msg_map.begin(); itr != sensor_msg_map.end(); ++itr) {
        candidate_.emplace_back(itr->second.front());
    }
    for (auto itr = sensor_msg_past.begin(); itr != sensor_msg_past.end(); ++itr) {
        itr->second.clear();
    }
}

template<typename AnySensor>
void ApproximateSynchronizer<AnySensor>::dequeMoveFrontToPast(const std::string index_frame_id) {
    auto &deque = sensor_msg_map.at(index_frame_id);
    auto &vector = sensor_msg_past.at(index_frame_id);
    if (deque.empty()) {
        RTHROW(name() + ": deque is empty!");
    }
    vector.push_back(deque.front());
    deque.pop_front();
    if (deque.empty()) {
        --num_non_empty_deques_;
    }
}

template<typename AnySensor>
void ApproximateSynchronizer<AnySensor>::publishCandidate() {
    for (const auto &cb : syn_cb_list_) {
        cb(candidate_);
    }
    candidate_.clear();
    pivot_ = NO_PIVOT;
    num_non_empty_deques_ = 0;
    recoverAndDelete();
}

template<typename AnySensor>
void ApproximateSynchronizer<AnySensor>::recoverAndDelete() {
    for (auto itr = sensor_msg_map.begin(); itr != sensor_msg_map.end(); ++itr) {
        const auto &frame_id = itr->first;
        auto &deque = sensor_msg_map.at(frame_id);
        auto &vector = sensor_msg_past.at(frame_id);
        while (!vector.empty()) {
            deque.push_front(vector.back());
            vector.pop_back();
        }

        if (deque.empty()) {
            RTHROW(name() + ": " + frame_id + " deque is empty!");
        }
        deque.pop_front();
        if (!deque.empty()) {
            ++num_non_empty_deques_;
        }
    }
}

template<typename AnySensor>
void ApproximateSynchronizer<AnySensor>::getVirtualCandidateStart(std::string &start_frame_id, uint64_t &start_time) {
    return getVirtualCandidateBoundary(start_frame_id, start_time, false);
}

template<typename AnySensor>
void ApproximateSynchronizer<AnySensor>::getVirtualCandidateEnd(std::string &end_frame_id, uint64_t &end_time) {
    return getVirtualCandidateBoundary(end_frame_id, end_time, true);
}

template<typename AnySensor>
void
ApproximateSynchronizer<AnySensor>::getVirtualCandidateBoundary(std::string &index_frame_id, uint64_t &time, bool end) {
    std::map<std::string, uint64_t> virtual_time_map;
    for (auto itr = sensor_msg_map.begin(); itr != sensor_msg_map.end(); ++itr) {
        const auto &frame_id = itr->first;
        virtual_time_map[frame_id] = getVirtualTime(frame_id);
    }
    auto begin_itr = virtual_time_map.begin();
    index_frame_id = begin_itr->first;
    time = begin_itr->second;
    for (auto itr = virtual_time_map.begin(); itr != virtual_time_map.end(); ++itr) {
        if ((itr->second < time) ^ end) {
            time = itr->second;
            index_frame_id = itr->first;
        }
    }
}

template<typename AnySensor>
uint64_t ApproximateSynchronizer<AnySensor>::getVirtualTime(const std::string &frame_id) {
    if (pivot_ == NO_PIVOT) {
        RTHROW(name() + ": pivot equal to NO_PIVOT!");
    }

    auto &vector = sensor_msg_past.at(frame_id);
    auto &deque = sensor_msg_map.at(frame_id);
    if (deque.empty()) {
        if (vector.empty()) {
            RTHROW(name() + ": " + frame_id + " past is empty!");
        }
        const auto &last_msg_time = vector.back()->getTimestamp();
        uint64_t msg_time_lower_bound = last_msg_time + inter_message_lower_bounds_.at(frame_id);
        if (msg_time_lower_bound > pivot_time_) {
            return msg_time_lower_bound;
        }
        return pivot_time_;
    }
    return deque.front()->getTimestamp();
}

template<typename AnySensor>
void ApproximateSynchronizer<AnySensor>::recover(const std::string &frame_id, int num_message) {
    auto &deque = sensor_msg_map.at(frame_id);
    auto &vector = sensor_msg_past.at(frame_id);

    if (num_message > static_cast<int>(vector.size())) {
        RTHROW(name() + ": num_message larger than past size!");
    }
    while (num_message > 0) {
        deque.push_front(vector.back());
        vector.pop_back();
        num_message--;
    }

    if (!deque.empty()) {
        ++num_non_empty_deques_;
    }
}

template<typename AnySensor>
void ApproximateSynchronizer<AnySensor>::recover() {
    for (auto itr = sensor_msg_map.begin(); itr != sensor_msg_map.end(); ++itr) {
        auto &deque = sensor_msg_map.at(itr->first);
        auto &vector = sensor_msg_past.at(itr->first);

        while (!vector.empty()) {
            deque.push_front(vector.back());
            vector.pop_back();
        }

        if (!deque.empty()) {
            ++num_non_empty_deques_;
        }
    }
}

template<typename AnySensor>
void ApproximateSynchronizer<AnySensor>::checkInterMessageBound(const std::string &frame_id) {
    auto &deque = sensor_msg_map.at(frame_id);
    auto &vector = sensor_msg_past.at(frame_id);

    if (deque.empty()) {
        RTHROW(name() + ": deque is empty!");
    }
    const auto &msg_time = deque.back()->getTimestamp();
    uint64_t previous_msg_time;
    if (deque.size() == (size_t) 1) {
        if (vector.empty()) {
            return;
        }
        previous_msg_time = vector.back()->getTimestamp();
    } else {
        previous_msg_time = deque[deque.size() - 2]->getTimestamp();
    }
    if (msg_time < previous_msg_time) {
        RWARN << name() << ": messages of frame_id " << frame_id << " arrived out of order!";
    } else if ((msg_time - previous_msg_time) < inter_message_lower_bounds_.at(frame_id)) {
        RWARN << name() << ": messages of frame_id " << frame_id << " arrived closer (" << msg_time
              << ") than the lower bound you provided (" << previous_msg_time << ")!";
    }
}

}  // namespace rally

#endif  // RALLY_UTILS_SYNCHRONIZER_IMPL_APPROXIMATE_SYNCHRONIZER_H
