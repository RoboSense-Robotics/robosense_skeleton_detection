//
// Created by sti on 2025/6/5.
//

#ifndef POSE_DETECTION_BASE_LIDAR_H
#define POSE_DETECTION_BASE_LIDAR_H

#include "rally/common/common.h"

namespace robosense {
namespace motion_capture {

struct LidarCalibOptions {
  rally::LidarEnum frame_id;
  Eigen::Matrix4f lidar_2_camera;
  Eigen::Matrix4f lidar_2_world;
};

class BaseLidar {
public:
  using Ptr = std::shared_ptr<BaseLidar>;

  virtual std::string name() = 0;

public:
  rally::LidarEnum getFrameId() { return frame_id_; }

  Eigen::Matrix4f getLidar2Camera() { return lidar_2_camera_; }

  Eigen::Matrix4f getLidar2World() { return lidar_2_world_; }

protected:
  rally::LidarEnum frame_id_;
  Eigen::Matrix4f lidar_2_camera_;
  Eigen::Matrix4f lidar_2_world_;
};

}
}



#endif //POSE_DETECTION_BASE_LIDAR_H
