//
// Created by sti on 2025/6/12.
//

#ifndef MOTION_CAPTURE_DATA_STRUCTURE_JOINT_POSE_H
#define MOTION_CAPTURE_DATA_STRUCTURE_JOINT_POSE_H

namespace robosense {
namespace motion_capture {

struct JointPose {
  using Ptr = std::shared_ptr<JointPose>;
  using ConstPtr = std::shared_ptr<const JointPose>;

  JointPose() : data(std::make_shared<std::vector<Eigen::Quaternionf>>()) {}

  uint64_t timestamp;
  std::shared_ptr<std::vector<Eigen::Quaternionf>> data;
};

struct JointPose2 {
  using Ptr = std::shared_ptr<JointPose2>;
  using ConstPtr = std::shared_ptr<const JointPose2>;

  JointPose2() : data(std::make_shared<std::vector<float>>()) {}

  uint64_t timestamp;
  std::shared_ptr<std::vector<float>> data;
};

}
}

#endif //MOTION_CAPTURE_DATA_STRUCTURE_JOINT_POSE_H
