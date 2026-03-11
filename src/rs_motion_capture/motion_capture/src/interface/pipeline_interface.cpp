//
// Created by sti on 2025/6/9.
//

#include "motion_capture/interface/pipeline_interface.h"

namespace robosense {
namespace motion_capture {

void PipelineInterface::init()
{
  spdlog::info("start init...");

  bool calib_mode = rally::ConfigureManager::getInstance().getCfgNode()["calib_mode"].as<bool>();
  bool check_mode = rally::ConfigureManager::getInstance().getCfgNode()["check_mode"].as<bool>();
  if (calib_mode || check_mode)
  {
    spdlog::info("Check/Calibration mode is enabled, set to OpticalMotionCapture.");
    pipeline_impl_ptr_.reset(BasePipelineRegister::getInstanceByName("OpticalMotionCapture"));
    pipeline_impl_ptr_->init();
    return;
  }
  auto pipeline_node = rally::ConfigureManager::getInstance().getCfgNode()["pipeline"];
  auto strategy = pipeline_node["strategy"].as<std::string>();
  if (!BasePipelineRegister::isValid(strategy))
  {
    const auto &all_instance = BasePipelineRegister::getAllInstances();
    spdlog::info("support strategy as follows");
    for (auto i : all_instance)
    {
      spdlog::info("{}", i->name());
    }
  }
  pipeline_impl_ptr_.reset(BasePipelineRegister::getInstanceByName(strategy));
  pipeline_impl_ptr_->init();
  spdlog::info("finish init.");
}

void PipelineInterface::start() {
  pipeline_impl_ptr_->start();
}

void PipelineInterface::stop() {
  pipeline_impl_ptr_->stop();
}

void PipelineInterface::addSensorData(const std::string& type, const PointCloud::Ptr& data) {
  pipeline_impl_ptr_->addSensorData(type, data);
}

void PipelineInterface::addSensorData(const std::string& type, const Image::Ptr& data) {
  pipeline_impl_ptr_->addSensorData(type, data);
}

void PipelineInterface::addSensorData(const std::string& type, const JointPose::Ptr& data) {
  pipeline_impl_ptr_->addSensorData(type, data);
}

void PipelineInterface::addSensorData(const std::string& type, const JointPose2::Ptr& data) {
  pipeline_impl_ptr_->addSensorData(type, data);
}

void PipelineInterface::addTrackerData(const std::string& type, const JointPose2::Ptr& data) {
  pipeline_impl_ptr_->addTrackerData(type, data);
}

void PipelineInterface::regOpticalCallback(const std::function<void(const Msg::Ptr&)>& callback) {
  pipeline_impl_ptr_->regOpticalCallback(callback);
}

void PipelineInterface::regInertialCallback(const std::function<void(const Msg::Ptr&)>& callback) {
  pipeline_impl_ptr_->regInertialCallback(callback);
}

}
}
