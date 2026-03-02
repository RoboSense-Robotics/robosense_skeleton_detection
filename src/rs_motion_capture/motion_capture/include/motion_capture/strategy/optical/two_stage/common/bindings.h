//
// Created by sti on 2025/6/10.
//

#ifndef MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_COMMON_BINDINGS_H
#define MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_COMMON_BINDINGS_H

#include <memory>
#include <string>
namespace robosense {
namespace motion_capture {

struct ObjectDetectionBindings {
  float* image = nullptr;
  float* output = nullptr;
};

struct PoseDetectionBindings {
  float* input = nullptr;
  float* simcc_x = nullptr;
  float* simcc_y = nullptr;
};

class Bindings {
public:
  using Ptr = std::shared_ptr<Bindings>;

  ObjectDetectionBindings od_bindings_host;
  ObjectDetectionBindings od_bindings_device;
  PoseDetectionBindings pd_bindings_host;
  PoseDetectionBindings pd_bindings_device;

  std::string name() const { return "Bindings"; }
};

}
}

#endif //MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_COMMON_BINDINGS_H
