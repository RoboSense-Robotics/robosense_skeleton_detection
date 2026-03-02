//
// Created by sti on 2025/7/29.
//

#ifndef INFRRARED_TRACKER_MOTION_CAPTURE_H
#define INFRRARED_TRACKER_MOTION_CAPTURE_H

#include "motion_capture/base/base_motion_capture.h"
#include <memory>
#include <vector>
namespace robosense{
namespace motion_capture{
class InfraredMotionCapture : public BaseMotionCapture{
public:
  using Ptr = std::shared_ptr<InfraredMotionCapture>;

  void init(const YAML::Node& cfg_node) override;

  void process(const Msg::Ptr& msg_ptr) override;

  std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, std::pair<Eigen::Vector3f, Eigen::Quaternionf>>
    calculateGlovePose(const Msg::Ptr &msg_ptr);
  std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, std::pair<Eigen::Vector3f, Eigen::Quaternionf>>
    passthroughTrackerPose(const Msg::Ptr &msg_ptr);

  std::string name() const override { return "InfraredMotionCapture"; }

private:
  TimeRecorder::Ptr time_recorder_ptr_;
  std::vector<float> left_spread_pose_;
  std::vector<float> right_spread_pose_;
  std::vector<float> person_zero_pose_;
  std::string glove_type_;
};
}
}

#endif
