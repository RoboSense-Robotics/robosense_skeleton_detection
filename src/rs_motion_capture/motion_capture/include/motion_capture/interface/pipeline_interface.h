//
// Created by sti on 2025/6/9.
//

#ifndef POSE_DETECTION_PIPELINE_INTERFACE_H
#define POSE_DETECTION_PIPELINE_INTERFACE_H

#include "motion_capture/base/base_pipeline.h"
#include <rs_log/init.h>

namespace robosense {
namespace motion_capture {

class PipelineInterface {
public:
  using Ptr = std::shared_ptr<PipelineInterface>;

  ~PipelineInterface() = default;

  void init();

  void start();

  void stop();

  void addSensorData(const std::string& type, const PointCloud::Ptr& data);

  void addSensorData(const std::string& type, const Image::Ptr& data);

  void addSensorData(const std::string& type, const JointPose::Ptr& data);

  void addSensorData(const std::string& type, const JointPose2::Ptr& data);
  void addTrackerData(const std::string& type, const JointPose2::Ptr& data);

  void regOpticalCallback(const std::function<void(const Msg::Ptr&)>& callback);

  void regInertialCallback(const std::function<void(const Msg::Ptr&)>& callback);

  std::string name() const { return "PipelineInterface"; }

private:
  BasePipeline::Ptr pipeline_impl_ptr_;
};

}
}

#endif //POSE_DETECTION_PIPELINE_INTERFACE_H
