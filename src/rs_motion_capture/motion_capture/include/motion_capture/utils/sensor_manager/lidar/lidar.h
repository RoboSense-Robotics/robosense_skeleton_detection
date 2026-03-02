//
// Created by sti on 2025/6/5.
//

#ifndef POSE_DETECTION_LIDAR_H
#define POSE_DETECTION_LIDAR_H

#include "motion_capture/utils/sensor_manager/lidar/base_lidar.h"

namespace robosense {
namespace motion_capture {
class Lidar : public BaseLidar {
public:
  using Ptr = std::shared_ptr<Lidar>;

  Lidar() = delete;

  Lidar(const LidarCalibOptions &options) {
    frame_id_ = options.frame_id;
    lidar_2_camera_ = options.lidar_2_camera;
    lidar_2_world_ = options.lidar_2_world;
  }

  std::string name() override { return "Lidar"; }
};
}
}

#endif //POSE_DETECTION_LIDAR_H
