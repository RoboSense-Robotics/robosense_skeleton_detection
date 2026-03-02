//
// Created by sti on 2025/6/5.
//

#ifndef POSE_DETECTION_TRANSFORM_H
#define POSE_DETECTION_TRANSFORM_H

#include "rally/common/common.h"

namespace robosense {
namespace motion_capture {

Eigen::Matrix4f quaternion_2_matrix(const Eigen::Quaternionf& rotation,
                                      const Eigen::Vector3f& translation);

}
}

#endif //POSE_DETECTION_TRANSFORM_H
