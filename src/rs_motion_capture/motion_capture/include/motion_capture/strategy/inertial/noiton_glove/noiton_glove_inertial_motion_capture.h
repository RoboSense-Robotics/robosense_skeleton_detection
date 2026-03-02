//
// Created by sti on 2025/6/12.
//

#ifndef MOTION_CAPTURE_STRATEGY_INERTIAL_NOITON_GLOVE_NOITON_GLOVE_INERTIAL_MOTION_CAPTURE_H
#define MOTION_CAPTURE_STRATEGY_INERTIAL_NOITON_GLOVE_NOITON_GLOVE_INERTIAL_MOTION_CAPTURE_H

#include "motion_capture/base/base_motion_capture.h"

namespace robosense {
namespace motion_capture {

class NoitonGloveInertialMotionCapture : public BaseMotionCapture {
public:
  using Ptr = std::shared_ptr<NoitonGloveInertialMotionCapture>;

  void init(const YAML::Node& cfg_node) override;

  void process(const Msg::Ptr& msg_ptr) override;

  std::string name() const override { return "NoitonGloveInertialMotionCapture"; }
};

}
}

#endif //MOTION_CAPTURE_STRATEGY_INERTIAL_NOITON_GLOVE_NOITON_GLOVE_INERTIAL_MOTION_CAPTURE_H
