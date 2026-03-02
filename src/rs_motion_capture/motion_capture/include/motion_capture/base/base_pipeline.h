//
// Created by sti on 2025/6/9.
//

#ifndef POSE_DETECTION_BASE_PIPELINE_H
#define POSE_DETECTION_BASE_PIPELINE_H

#include "motion_capture/common/message.h"
#include "motion_capture/data_structure/any_msg.h"
#include "motion_capture/data_structure/image.h"
#include "motion_capture/data_structure/point_cloud.h"
#include "motion_capture/data_structure/joint_pose.h"

namespace robosense {
namespace motion_capture {

class BasePipeline {
public:
  using Ptr = std::shared_ptr<BasePipeline>;

  virtual ~BasePipeline() = default;

  virtual void init() = 0;

  virtual void start() = 0;

  virtual void stop() = 0;

  virtual void addSensorData(const std::string& type, const PointCloud::Ptr& data) = 0;

  virtual void addSensorData(const std::string& type, const Image::Ptr& data) = 0;

  virtual void addSensorData(const std::string& type, const JointPose::Ptr& data) = 0;

  virtual void addSensorData(const std::string &type, const JointPose2::Ptr &data) = 0;

  virtual void addTrackerData(const std::string &type, const JointPose2::Ptr &data) = 0;

  virtual void regOpticalCallback(const std::function<void(const Msg::Ptr&)>& callback) = 0;

  virtual void regInertialCallback(const std::function<void(const Msg::Ptr&)>& callback) = 0;

  virtual std::string name() const = 0;
};

RALLY_REGISTER_REGISTER(BasePipeline);
#define RS_REGISTER_PIPELINE(name) RALLY_REGISTER_CLASS(BasePipeline, name)

}
}

#endif //POSE_DETECTION_BASE_PIPELINE_H
