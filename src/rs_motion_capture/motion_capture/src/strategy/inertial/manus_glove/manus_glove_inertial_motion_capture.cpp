//
// Created by sti on 2025/7/2.
//

#include "motion_capture/strategy/inertial/manus_glove/manus_glove_inertial_motion_capture.h"

namespace robosense {
namespace motion_capture {

void ManusGloveInertialMotionCapture::init(const YAML::Node& cfg_node) {

}

void ManusGloveInertialMotionCapture::process(const Msg::Ptr& msg_ptr) {

}


RS_REGISTER_MOTION_CAPTURE(ManusGloveInertialMotionCapture);

}
}