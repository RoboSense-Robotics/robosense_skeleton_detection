//
// Created by sti on 2025/6/6.
//

#include "motion_capture/strategy/optical/two_stage/pose_detection/preprocess.h"
#include "motion_capture/utils/sensor_manager/sensor_manager.h"
#include "motion_capture/algorithm/image_process/image_process.h"

namespace robosense {
namespace motion_capture {

void PoseDetectionPreprocess::init(const YAML::Node& cfg_node) {
  AINFO << name() << ": start init...";
  img_attr_.image_width_ = SensorManager::getInstance().getWidth(rally::CameraEnum::left_ac_camera);
  img_attr_.image_height_ = SensorManager::getInstance().getHeight(rally::CameraEnum::left_ac_camera);
  img_attr_.half_width_ = img_attr_.image_width_ / 2;
  img_attr_.half_height_ = img_attr_.image_height_ / 2;
  img_attr_.model_input_width_ = 288;
  img_attr_.model_input_height_ = 384;
  aspect_ratio_ = static_cast<float>(img_attr_.model_input_width_) / img_attr_.model_input_height_;
  time_recorder_ptr_ = std::make_shared<TimeRecorder>(name());
  AINFO << name() << ": finish init.";
}

void PoseDetectionPreprocess::process(const Msg::Ptr &msg_ptr) {
  time_recorder_ptr_->tic();
  AINFO << name() << ": start process...";
  if (msg_ptr->internal_result_ptr->is_person_detected_map[rally::CameraEnum::left_ac_camera] == 0 ||
      msg_ptr->internal_result_ptr->is_person_detected_map[rally::CameraEnum::right_ac_camera] == 0) {
    if (msg_ptr->internal_result_ptr->is_person_detected_map[rally::CameraEnum::left_ac_camera] == 0) {
      RWARN << name() << ": left ac camera not detected, skipping pose detection!";
    }
    if (msg_ptr->internal_result_ptr->is_person_detected_map[rally::CameraEnum::right_ac_camera] == 0) {
      RWARN << name() << ": right ac camera not detected, skipping pose detection!";
    }
    return;
  }

  process_single_image(msg_ptr, rally::CameraEnum::left_ac_camera);
  process_single_image(msg_ptr, rally::CameraEnum::right_ac_camera);
  cudaStreamSynchronize(stream_);
  AINFO << name() << ": finish process.";
  time_recorder_ptr_->toc();
}

void PoseDetectionPreprocess::process_single_image(const Msg::Ptr &msg_ptr, rally::CameraEnum camera_enum) {
  const auto& xmin = msg_ptr->internal_result_ptr->detected_person_bbox_map[camera_enum].xmin;
  const auto& ymin = msg_ptr->internal_result_ptr->detected_person_bbox_map[camera_enum].ymin;
  const auto& xmax = msg_ptr->internal_result_ptr->detected_person_bbox_map[camera_enum].xmax;
  const auto& ymax = msg_ptr->internal_result_ptr->detected_person_bbox_map[camera_enum].ymax;

  float box_center_x = (xmin + xmax) * 0.5f;
  float box_center_y = (ymin + ymax) * 0.5f;
  float box_width = (xmax - xmin) * expand_ratio_;
  float box_height = (ymax - ymin) * expand_ratio_;
  if (box_width > box_height * aspect_ratio_) {
    // 宽度更宽的情况，保持宽度不变，调整高度
    box_height = box_width / aspect_ratio_;
  } else {
    // 高度更高的情况，保持高度不变，调整宽度
    box_width = box_height * aspect_ratio_;
  }

  // 保存box信息
  BBox<float> bbox(box_center_x + box_width * 0.5f,
                   box_center_x - box_width * 0.5f,
                   box_center_y + box_height * 0.5f,
                   box_center_y - box_height * 0.5f,
                    0.f);
  msg_ptr->internal_result_ptr->pose_model_input_bbox_map[camera_enum] = bbox;

  float scale = img_attr_.model_input_width_ / box_width;
  uint32_t scaled_width = static_cast<uint32_t>(img_attr_.image_width_ * scale);
  uint32_t scaled_height = static_cast<uint32_t>(img_attr_.image_height_ * scale);
  FillColor fillcolor;
  fillcolor.color[0] = 0;
  fillcolor.color[1] = 0;
  fillcolor.color[2] = 0;
  int x_output_offset = -static_cast<int>((box_center_x - box_width * 0.5f) * scale);
  int y_output_offset = -static_cast<int>((box_center_y - box_height * 0.5f) * scale);

  uint8_t* src_ptr = camera_enum == rally::CameraEnum::left_ac_camera ?
                     msg_ptr->internal_result_ptr->undistorted_image_ptr :
                     msg_ptr->internal_result_ptr->undistorted_image_ptr + img_attr_.image_width_ * img_attr_.image_height_ * 3;
  float* dst_ptr = camera_enum == rally::CameraEnum::left_ac_camera ?
                   bindings_ptr_->pd_bindings_device.input :
                   bindings_ptr_->pd_bindings_device.input + img_attr_.model_input_width_ * img_attr_.model_input_height_ * 3;

  batched_image_universal_kernel_conversion(
          src_ptr,
          img_attr_.image_width_,
          img_attr_.image_width_ * 3,
          img_attr_.image_height_,
          1,
          scaled_width,
          scaled_height,
          x_output_offset,
          y_output_offset,
          fillcolor,
          dst_ptr,
          img_attr_.model_input_width_,
          img_attr_.model_input_width_,
          img_attr_.model_input_height_,
          DataType::Float32,
          PixelLayout::NCHW_RGB,
          Interpolation::Bilinear,
          img_attr_.mean_rgb[0],
          img_attr_.mean_rgb[1],
          img_attr_.mean_rgb[2],
          img_attr_.std_rgb[0],
          img_attr_.std_rgb[1],
          img_attr_.std_rgb[2],
          nullptr,
          stream_
  );


//  // 可視化
//  cudaStreamSynchronize(stream_);
//  std::vector<float> data_ptr(3 * img_attr_.model_input_height_ * img_attr_.model_input_width_);
//
//  cudaMemcpy(data_ptr.data(), dst_ptr,
//             3 * img_attr_.model_input_height_ * img_attr_.model_input_width_ * sizeof(float),
//             cudaMemcpyDeviceToHost);
//  int height = img_attr_.model_input_height_;
//  int width = img_attr_.model_input_width_;
//  int channels = 3;
//  cv::Mat image(height, width, CV_8UC3);  // 8-bit, 3 channels (BGR)
//  for (int h = 0; h < height; ++h) {
//    for (int w = 0; w < width; ++w) {
//      for (int c = 0; c < channels; ++c) {
//        // 计算数据索引 (2×3×H×W → H×W×3)
//        int idx = c * (height * width) + h * width + w;
//
//        // 反归一化: x = (x * std) + mean
//        float pixel_val = data_ptr[idx] * img_attr_.std_rgb[c] + img_attr_.mean_rgb[c];
//
//        // 裁剪到 [0, 1] 并缩放到 [0, 255]
//        pixel_val = std::max(0.0f, std::min(1.0f, pixel_val));
//        image.at<cv::Vec3b>(h, w)[c] = static_cast<uchar>(pixel_val * 255.0f);
//      }
//    }
//  }
//  // 4. 显示图像
//  cv::imwrite("/mnt/1T/tmp/image_" + std::to_string(msg_ptr->timestamp) + rally::kCameraEnum2NameMap.at(camera_enum) + ".png", image);  // 可选：保存到文件

}

}
}
