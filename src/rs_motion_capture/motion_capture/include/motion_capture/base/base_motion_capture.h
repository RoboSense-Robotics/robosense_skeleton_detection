//
// Created by sti on 2025/6/6.
//

#ifndef POSE_DETECTION_BASE_MOTION_CAPTURE_H
#define POSE_DETECTION_BASE_MOTION_CAPTURE_H

#include "motion_capture/common/message.h"

namespace robosense {
namespace motion_capture {

class BaseMotionCapture {
public:
  using Ptr = std::shared_ptr<BaseMotionCapture>;

  virtual ~BaseMotionCapture() = default;

  virtual void init(const YAML::Node& cfg_node) = 0;

  virtual void process(const Msg::Ptr& msg_ptr) = 0;

  virtual std::string name() const = 0;
};

RALLY_REGISTER_REGISTER(BaseMotionCapture);
#define RS_REGISTER_MOTION_CAPTURE(name) RALLY_REGISTER_CLASS(BaseMotionCapture, name)

}
}

#endif //POSE_DETECTION_BASE_MOTION_CAPTURE_H
