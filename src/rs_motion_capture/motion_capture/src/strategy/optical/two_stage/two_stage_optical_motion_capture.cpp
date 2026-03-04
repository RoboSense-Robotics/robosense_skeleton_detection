//
// Created by sti on 2025/6/6.
//

#include "motion_capture/strategy/optical/two_stage/two_stage_optical_motion_capture.h"

namespace robosense {
namespace motion_capture {

void TwoStageOpticalMotionCapture::init(const YAML::Node& cfg_node) {
  spdlog::info("start init...");
  YAML::Node od_cfg_node = cfg_node["object_detection"];
  YAML::Node pd_cfg_node = cfg_node["pose_detection"];
  YAML::Node fusion_cfg_node = cfg_node["fusion"];
  YAML::Node check_cfg_node = cfg_node["check"];
  // create stream
  if (cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking) != cudaSuccess) {
    RTHROW("Failed to create CUDA stream");
  }
  // bindings
  bindings_ptr_ = std::make_shared<Bindings>();
  // init
  object_detection_ptr_ = std::make_shared<ObjectDetection>(bindings_ptr_, stream_);
  object_detection_ptr_->init(od_cfg_node);
  pose_detection_ptr_ = std::make_shared<PoseDetection>(bindings_ptr_, stream_);
  pose_detection_ptr_->init(pd_cfg_node);

  calib_mode_ = rally::ConfigureManager::getInstance().getCfgNode()["calib_mode"].as<bool>();
  check_mode_ = rally::ConfigureManager::getInstance().getCfgNode()["check_mode"].as<bool>();
  if (check_mode_) {
    check_calibration_ptr_ = std::make_shared<CheckCalibration>();
    check_calibration_ptr_->init(check_cfg_node);
  } else if (calib_mode_) {
    bone_calibration_ptr_ = std::make_shared<BoneCalibration>();
    bone_calibration_ptr_->init();
  } else {
    pose_fusion_optimization_ptr_ = std::make_shared<PoseFusionOptimization>();
    pose_fusion_optimization_ptr_->init(fusion_cfg_node);
  }

  time_recorder_ptr_ = std::make_shared<TimeRecorder>("TwoStageOpticalMotionCapture");

  spdlog::info("finish init.");
}

void TwoStageOpticalMotionCapture::process(const Msg::Ptr& msg_ptr) {
  time_recorder_ptr_->tic();
  spdlog::info("start process...");
  // Process object detection
  object_detection_ptr_->process(msg_ptr);

  // Process pose detection
  pose_detection_ptr_->process(msg_ptr);

  // Perform fusion optimization
  if (check_mode_) {
    check_calibration_ptr_->process(msg_ptr);
  } else if (calib_mode_) {
    bone_calibration_ptr_->process(msg_ptr);
  } else {
    pose_fusion_optimization_ptr_->process(msg_ptr);
  }
  spdlog::info("finish process.");
  time_recorder_ptr_->toc();
}

RS_REGISTER_MOTION_CAPTURE(TwoStageOpticalMotionCapture);

}
}

