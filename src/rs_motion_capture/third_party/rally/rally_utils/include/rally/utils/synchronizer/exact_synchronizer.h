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
#ifndef RALLY_UTILS_SYNCHRONIZER_EXACT_SYNCHRONIZER_H
#define RALLY_UTILS_SYNCHRONIZER_EXACT_SYNCHRONIZER_H

#include <map>
#include <set>
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <functional>
#include "rally/utils/utils.h"
#include "rally/utils/synchronizer/details/details.h"

namespace rally {

struct ExactSynOptions {
    uint16_t queue_size = 10;
    std::vector<std::string> frame_id_vec;
};

template<typename AnySensor>
class ExactSynchronizer {
    RALLY_STATIC_ASSERT(details::has_getTimestamp<AnySensor>().value && details::has_getFrameID<AnySensor>().value);
public:
    using Ptr = std::unique_ptr<ExactSynchronizer>;
    using AnySensorPtr = std::shared_ptr<AnySensor>;

    explicit ExactSynchronizer(const ExactSynOptions &options = ExactSynOptions());

    void addData(const AnySensorPtr &msg_ptr);

    void regSynCallback(const std::function<void(const std::vector<AnySensorPtr> &msg_ptr_vec)> &cb);

private:
    std::string name() {
        return "ExactSynchronizer";
    }

    void clearOldQueue();

    uint64_t last_signal_time_;
    std::mutex mutex_, reg_mutex_;
    std::map<uint64_t, std::map<std::string, AnySensorPtr> > queues_;

    std::vector<std::function<void(const std::vector<AnySensorPtr> &msg_ptr_vec)> > syn_cb_list_;

    ExactSynOptions options_;
    std::set<std::string> frame_id_set_;
};

}  // namespace rally

#include "rally/utils/synchronizer/impl/exact_synchronizer.h"

#endif  // RALLY_UTILS_SYNCHRONIZER_EXACT_SYNCHRONIZER_H
