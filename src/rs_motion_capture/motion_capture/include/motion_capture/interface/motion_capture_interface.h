//
// Created by sti on 2025/6/9.
//

#ifndef MOTION_CAPTURE_INTERFACE_MOTION_CAPTURE_INTERFACE_H
#define MOTION_CAPTURE_INTERFACE_MOTION_CAPTURE_INTERFACE_H

#include "motion_capture/base/base_motion_capture.h"
#include <spdlog/spdlog.h>

namespace robosense {
namespace motion_capture {

class MotionCaptureInterface {
public:
  using Ptr = std::shared_ptr<MotionCaptureInterface>;
  using ConstPtr = std::shared_ptr<const MotionCaptureInterface>;

  /// @brief init
  void init(const YAML::Node& cfg_node);

  /// @brief process
  void process(const Msg::Ptr& msg_ptr);

  /// @brief name
  std::string name() const { return "MotionCaptureInterface"; }

private:
  BaseMotionCapture::Ptr motion_capture_impl_ptr_;
};

}
}

#endif //MOTION_CAPTURE_INTERFACE_MOTION_CAPTURE_INTERFACE_H
