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
#ifndef RALLY_UTILS_SYNCHRONIZER_APPROXIMATE_SYNCHRONIZER_H
#define RALLY_UTILS_SYNCHRONIZER_APPROXIMATE_SYNCHRONIZER_H

#include <vector>
#include <string>
#include <deque>
#include <map>
#include <thread>
#include <condition_variable>
#include <functional>
#include "rally/utils/utils.h"
#include "rally/utils/synchronizer/details/details.h"

namespace rally {

struct ApproximateSynOptions {
    std::vector<uint32_t> queue_size;
    std::vector<std::string> frame_id_vec;
    uint64_t max_interval_duration = std::numeric_limits<uint64_t>::max();
    /// lower_bound, Usually less than the period of the lowest frequency sensor,
    /// unit: ms
    std::vector<uint64_t> inter_message_lower_bound;
    float age_penalty = 0.f;
};

template<typename AnySensor>
class ApproximateSynchronizer {
    RALLY_STATIC_ASSERT(details::has_getTimestamp<AnySensor>::value && details::has_getFrameID<AnySensor>::value);
public:
    using Ptr = std::unique_ptr<ApproximateSynchronizer>;
    using AnySensorPtr = std::shared_ptr<AnySensor>;

    explicit ApproximateSynchronizer(const ApproximateSynOptions &options = ApproximateSynOptions());

    void addData(const AnySensorPtr &msg_ptr);

    void start();

    void stop();

    ~ApproximateSynchronizer();

    const std::unique_ptr<std::thread> &getTd() const;

    void regSynCallback(const std::function<void(const std::vector<AnySensorPtr> &msg_ptr_vec)> &cb);

private:
    const std::string name() const {
        return "ApproximateSynchronizer";
    }

    void core();

    void process();

    void getCandidateEnd(std::string &end_frame_id, uint64_t &end_time);

    void getCandidateStart(std::string &start_frame_id, uint64_t &start_time);

    void getCandidateBoundary(std::string &frame_id, uint64_t &time, bool end);

    void dequeDeleteFront(const std::string &frame_id);

    void makeCandidate();

    void dequeMoveFrontToPast(const std::string index_frame_id);

    void publishCandidate();

    void recoverAndDelete();

    void getVirtualCandidateStart(std::string &start_frame_id, uint64_t &start_time);

    void getVirtualCandidateEnd(std::string &end_frame_id, uint64_t &end_time);

    void getVirtualCandidateBoundary(std::string &index_frame_id, uint64_t &time, bool end);

    uint64_t getVirtualTime(const std::string &frame_id);

    void recover(const std::string &frame_id, int num_message);

    void recover();

    void checkInterMessageBound(const std::string &frame_id);

    ApproximateSynOptions options_;
    std::unique_ptr<std::thread> thread_ptr_;
    std::mutex mutex_;
    std::condition_variable condition_;
    bool run_flag_ = false;
    std::deque<AnySensorPtr> msg_deque_;

    std::vector<std::function<void(const std::vector<AnySensorPtr> &msg_ptr_vec)>> syn_cb_list_;

    std::map<std::string, uint32_t> queue_size_;
    std::map<std::string, std::deque<AnySensorPtr> > sensor_msg_map;
    std::map<std::string, std::vector<AnySensorPtr> > sensor_msg_past;
    std::map<std::string, bool> has_dropped_messages_;
    std::map<std::string, uint64_t> inter_message_lower_bounds_;

    uint32_t num_non_empty_deques_;
    std::string pivot_;
    const std::string NO_PIVOT = "_no_pivot";
    std::vector<AnySensorPtr> candidate_;
    uint64_t candidate_start_, candidate_end_, pivot_time_;
    float age_penalty_ = 0.f;
};

}  // namespace rally

#include "rally/utils/synchronizer/impl/approximate_synchronizer.h"

#endif  // RALLY_UTILS_SYNCHRONIZER_APPROXIMATE_SYNCHRONIZER_H
