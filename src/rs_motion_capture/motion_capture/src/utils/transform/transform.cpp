//
// Created by sti on 2025/6/5.
//

#include "motion_capture/utils/transform/transform.h"

namespace robosense {
namespace motion_capture {

Eigen::Matrix4f quaternion_2_matrix(const Eigen::Quaternionf& rotation,
                                    const Eigen::Vector3f& translation) {
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  matrix.block<3,3>(0,0) = rotation.normalized().toRotationMatrix();
  matrix.block<3,1>(0,3) = translation;
  return matrix;
}

}
}