//
// Created by sti on 2025/6/12.
//

#ifndef MOTION_CAPTURE_DATA_STRUCTURE_POINT_CLOUD_H
#define MOTION_CAPTURE_DATA_STRUCTURE_POINT_CLOUD_H

#include "rally/utils/utils.h"

namespace robosense {
namespace motion_capture {

struct PointCloud {
public:
  using Ptr = std::shared_ptr<PointCloud>;
  using ConstPtr = std::shared_ptr<const PointCloud>;

  PointCloud() {
    data.reset(new pcl::PointCloud<pcl::PointXYZ>());
  }

  uint64_t timestamp;
  pcl::PointCloud<pcl::PointXYZ>::Ptr data;
};

}
}

#endif //MOTION_CAPTURE_DATA_STRUCTURE_POINT_CLOUD_H
