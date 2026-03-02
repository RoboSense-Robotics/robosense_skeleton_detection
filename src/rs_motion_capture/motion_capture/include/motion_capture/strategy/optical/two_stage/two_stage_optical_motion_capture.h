//
// Created by sti on 2025/6/6.
//

#ifndef POSE_DETECTION_TWO_STAGE_OPTICAL_MOTION_CAPTURE_H
#define POSE_DETECTION_TWO_STAGE_OPTICAL_MOTION_CAPTURE_H

#include "motion_capture/base/base_motion_capture.h"
#include "motion_capture/strategy/optical/two_stage/pose_detection/pose_detection.h"
#include "motion_capture/strategy/optical/two_stage/object_detection/object_detection.h"
#include "motion_capture/strategy/optical/two_stage/fusion/fusion.h"
#include "motion_capture/strategy/optical/two_stage/bone_calibration/calibration.h"
#include "motion_capture/strategy/optical/two_stage/check_calibration/check_calibration.h"

namespace robosense {
namespace motion_capture {

class TwoStageOpticalMotionCapture : public BaseMotionCapture {
public:
  using Ptr = std::shared_ptr<TwoStageOpticalMotionCapture>;

  void init(const YAML::Node& cfg_node) override;

  void process(const Msg::Ptr& msg_ptr) override;

  std::string name() const override { return "TwoStageOpticalMotionCapture"; }

private:
  ObjectDetection::Ptr object_detection_ptr_;
  PoseDetection::Ptr pose_detection_ptr_;
  PoseFusionOptimization::Ptr pose_fusion_optimization_ptr_;
  BoneCalibration::Ptr bone_calibration_ptr_;
  CheckCalibration::Ptr check_calibration_ptr_;

  cudaStream_t stream_;
  Bindings::Ptr bindings_ptr_;
  bool calib_mode_;
  bool check_mode_;

private:
  TimeRecorder::Ptr time_recorder_ptr_;
};

}
}

#endif //POSE_DETECTION_TWO_STAGE_OPTICAL_MOTION_CAPTURE_H
