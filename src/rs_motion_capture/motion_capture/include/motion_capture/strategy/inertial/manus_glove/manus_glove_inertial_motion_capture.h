//
// Created by sti on 2025/7/2.
//

#ifndef MOTION_CAPTURE_STRATEGY_INERTIAL_MANUS_GLOVE_MANUS_GLOVE_INERTIAL_MOTION_CAPTURE_H
#define MOTION_CAPTURE_STRATEGY_INERTIAL_MANUS_GLOVE_MANUS_GLOVE_INERTIAL_MOTION_CAPTURE_H

#include "motion_capture/base/base_motion_capture.h"

namespace robosense {
namespace motion_capture {

class ManusGloveInertialMotionCapture : public BaseMotionCapture {
public:
  using Ptr = std::shared_ptr<ManusGloveInertialMotionCapture>;

  void init(const YAML::Node& cfg_node) override;

  void process(const Msg::Ptr& msg_ptr) override;

  std::string name() const override { return "ManusGloveInertialMotionCapture"; }
};

}
}

#endif //MOTION_CAPTURE_STRATEGY_INERTIAL_MANUS_GLOVE_MANUS_GLOVE_INERTIAL_MOTION_CAPTURE_H
