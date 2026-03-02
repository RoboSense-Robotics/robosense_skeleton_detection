//
// Created by sti on 2025/6/6.
//

#ifndef MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_POSE_DETECTION_POSE_DETECTION_H
#define MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_POSE_DETECTION_POSE_DETECTION_H

#include "motion_capture/common/message.h"
#include "motion_capture/strategy/optical/two_stage/pose_detection/preprocess.h"
#include "motion_capture/strategy/optical/two_stage/pose_detection/postprocess.h"
#include "hyper_vision/inference/inference.h"

namespace robosense {
namespace motion_capture {

class PoseDetection {
public:
  using Ptr = std::shared_ptr<PoseDetection>;

  PoseDetection(const Bindings::Ptr& bindings_ptr, cudaStream_t stream)
      : bindings_ptr_(bindings_ptr), stream_(stream) {}

  void init(const YAML::Node& cfg_node);

  void process(const Msg::Ptr &msg_ptr);

  std::string name() const { return "PoseDetection"; }

private:
  robosense::inference::InferEngine::Ptr infer_ptr_;
  PoseDetectionPreprocess::Ptr preprocess_ptr_;
  PoseDetectionPostprocess::Ptr postprocess_ptr_;
  Bindings::Ptr bindings_ptr_;
  cudaStream_t stream_;

private:
  TimeRecorder::Ptr time_recorder_ptr_;
  TimeRecorder::Ptr infer_time_recorder_ptr_;
};

}
}

#endif //MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_POSE_DETECTION_POSE_DETECTION_H
