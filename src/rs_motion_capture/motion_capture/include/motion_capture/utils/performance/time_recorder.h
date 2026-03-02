//
// Created by sti on 2025/6/6.
//

#ifndef POSE_DETECTION_TIME_RECORDER_H
#define POSE_DETECTION_TIME_RECORDER_H

#include <algorithm>
#include <chrono>
#include <limits>
#include <string>
#include <iomanip>
#include "rally/utils/utils.h"
#include <rs_log/init.h>

namespace robosense {
namespace motion_capture {

class TimeRecorder {
public:
  using Ptr = std::shared_ptr<TimeRecorder>;
  TimeRecorder() = default;
  explicit TimeRecorder(std::string title) : title_(title) {}
  void init(std::string title) { title_ = title; }

  void tic() { start_point_ = std::chrono::high_resolution_clock::now(); }

  void toc() {
    if (global_cnt < 10) {
      global_cnt++;
      return;
    }
    // 1. cur time
    end_point_ = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds time_nano =
            std::chrono::duration_cast<std::chrono::nanoseconds>(end_point_ -
                                                                 start_point_);
    float time_micro = time_nano.count() / 1e6;

    // 2. max min
    max_val = std::max(time_micro, max_val);
    min_val = std::min(time_micro, min_val);

    // 3. mean
    sum_val += time_micro;
    cnt += 1;
    mean_val = sum_val / cnt;

    // 4. std
    if (cnt % 1 == 0) {
      AINFO << std::fixed << std::setprecision(3) << title_ << " cnt : " << cnt << ": min : " << min_val
              << " ms, max : " << max_val << " ms, "
              << "mean : " << mean_val << " ms, cur : " << time_micro << " ms";
    }
  }

  void lat(uint64_t sensor_timestamp) {
    if (latancy_global_cnt < 10) {
      latancy_global_cnt++;
      return;
    }
    end_point_ = std::chrono::high_resolution_clock::now();
    auto sensor_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::nanoseconds(sensor_timestamp));
    auto sensor_pt = std::chrono::time_point<std::chrono::high_resolution_clock>(sensor_duration);
    std::chrono::nanoseconds time_nano =
            std::chrono::duration_cast<std::chrono::nanoseconds>(end_point_ - sensor_pt);
    double time_micro = time_nano.count() / 1e6;

    // 2. max min
    latancy_max_val = std::max(time_micro, latancy_max_val);
    latancy_min_val = std::min(time_micro, latancy_min_val);

    // 3. mean
    latancy_sum_val += time_micro;
    latancy_cnt += 1;
    latancy_mean_val = latancy_sum_val / latancy_cnt;
    AINFO << std::fixed << std::setprecision(3) << title_ << " cnt : " << latancy_cnt << ": min : " << latancy_min_val
            << " ms, max : " << latancy_max_val << " ms, "
            << "mean : " << latancy_mean_val << " ms, cur : " << time_micro << " ms, trigger msg ts: " << rally::toSeconds(sensor_timestamp);
  }

public:
  std::string title_;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_point_,
          end_point_;
  int cnt = 0, global_cnt = 0;
  float max_val = std::numeric_limits<float>::min();
  float min_val = std::numeric_limits<float>::max();
  float sum_val = 0.;
  float mean_val = 0.;
  int latancy_cnt = 0, latancy_global_cnt = 0;
  double latancy_max_val = std::numeric_limits<double>::min();
  double latancy_min_val = std::numeric_limits<double>::max();
  double latancy_sum_val = 0.;
  double latancy_mean_val = 0.;
};

}
}

#endif //POSE_DETECTION_TIME_RECORDER_H
