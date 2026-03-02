//
// Created by sti on 2025/6/6.
//

#ifndef MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_POSE_DETECTION_PREPROCESS_H
#define MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_POSE_DETECTION_PREPROCESS_H

#include <cuda_runtime.h>
#include "motion_capture/common/message.h"
#include "motion_capture/strategy/optical/two_stage/common/bindings.h"

namespace robosense {
namespace motion_capture {

class PoseDetectionPreprocess {
public:
  using Ptr = std::shared_ptr<PoseDetectionPreprocess>;

  PoseDetectionPreprocess(const Bindings::Ptr &bindings_ptr, cudaStream_t stream)
      : bindings_ptr_(bindings_ptr), stream_(stream) {}

  void init(const YAML::Node& cfg_node);

  void process(const Msg::Ptr &msg_ptr);

  std::string name() const { return "PoseDetectionPreprocess"; }

private:
  void process_single_image(const Msg::Ptr &msg_ptr, rally::CameraEnum camera_enum);

private:
  struct ImgAttr {
    uint8_t batch_size_ = 2;
    uint32_t image_width_;
    uint32_t image_height_;
    uint32_t rgb_img_channel_ = 3;
    size_t single_rgb_img_size_;
    size_t double_depth_img_size_;
    std::vector<float> mean_rgb = {123.6750 / 255.f, 116.2800 / 255.f, 103.5300 / 255.f};
    std::vector<float> std_rgb = {58.3950 / 255.f, 57.1200 / 255.f, 57.3750 / 255.f};
    uint32_t model_input_width_;
    uint32_t model_input_height_;
    float resize_scale_;
    uint32_t scaled_width_;
    uint32_t scaled_height_;
    uint32_t output_x_offset_;
    uint32_t output_y_offset_;
    uint32_t half_width_;
    uint32_t half_height_;
  } img_attr_;

  float expand_ratio_ = 1.25f;
  float aspect_ratio_;

  //
  Bindings::Ptr bindings_ptr_;
  cudaStream_t stream_;

private:
  TimeRecorder::Ptr time_recorder_ptr_;
};

}
}

#endif //MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_POSE_DETECTION_PREPROCESS_H
