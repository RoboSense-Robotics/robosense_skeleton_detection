//
// Created by sti on 2025/6/6.
//

#include "motion_capture/strategy/optical/two_stage/object_detection/preprocess.h"
#include "motion_capture/common/macros.h"
#include "motion_capture/utils/sensor_manager/sensor_manager.h"
#include "motion_capture/algorithm/image_process/image_process.h"
#include "rally/utils/global/configure_manager.h"
#include <cuda_runtime.h>

namespace robosense {
namespace motion_capture {

void ObjectDetectionPreprocess::init(const YAML::Node& cfg_node) {
  AINFO << name() << ": start init...";
  // 获取畸变map
  img_attr_.image_width_ = SensorManager::getInstance().getWidth(rally::CameraEnum::left_ac_camera);
  img_attr_.image_height_ = SensorManager::getInstance().getHeight(rally::CameraEnum::left_ac_camera);
  size_t remap_xy_size = img_attr_.batch_size_ * img_attr_.image_width_ * img_attr_.image_height_ * 2 * sizeof(float);
#ifdef __aarch64__
  CUDA_CHECK(cudaMallocHost((void**)&remap_xy_, remap_xy_size));
#else
  CUDA_CHECK(cudaMalloc((void**)&remap_xy_, remap_xy_size));
#endif
  cv::Mat left_remap_x, left_remap_y, right_remap_x, right_remap_y;
  SensorManager::getInstance().getRectifyMap(rally::CameraEnum::left_ac_camera, img_attr_.image_width_, img_attr_.image_height_, left_remap_x, left_remap_y);
  SensorManager::getInstance().getRectifyMap(rally::CameraEnum::right_ac_camera, img_attr_.image_width_, img_attr_.image_height_, right_remap_x, right_remap_y);
#ifdef __aarch64__
  std::memcpy(remap_xy_, left_remap_x.ptr<float>(), remap_xy_size / 2);
  std::memcpy(remap_xy_ + img_attr_.image_width_ * img_attr_.image_height_ * 2, right_remap_x.ptr<float>(), remap_xy_size / 2);
#else
  CUDA_CHECK(cudaMemcpy(remap_xy_, left_remap_x.ptr<float>(), remap_xy_size / 2, cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(remap_xy_ + img_attr_.image_width_ * img_attr_.image_height_ * 2, right_remap_x.ptr<float>(), remap_xy_size / 2, cudaMemcpyHostToDevice));
#endif
  // 初始化去畸变图像指针
  img_attr_.single_rgb_img_size_ = img_attr_.image_width_ * img_attr_.image_height_ * 3 * sizeof(uint8_t);
  img_attr_.double_depth_img_size_ = img_attr_.batch_size_ * img_attr_.single_rgb_img_size_;
#ifdef __aarch64__
  CUDA_CHECK(cudaMallocHost((void**)&undistort_img_ptr_, img_attr_.double_depth_img_size_));
  CUDA_CHECK(cudaMallocHost((void**)&distort_img_ptr_, img_attr_.double_depth_img_size_));
#else
  CUDA_CHECK(cudaMalloc((void**)&undistort_img_ptr_, img_attr_.double_depth_img_size_));
  CUDA_CHECK(cudaMalloc((void**)&distort_img_ptr_, img_attr_.double_depth_img_size_));
#endif
  // 初始化1/2去畸变图像指针
  img_attr_.half_width_ = img_attr_.image_width_ / 2;
  img_attr_.half_height_ = img_attr_.image_height_ / 2;
#ifdef __aarch64__
  CUDA_CHECK(cudaMallocHost((void**)&half_undistort_img_ptr_,img_attr_.double_depth_img_size_ / 4));
#else
  CUDA_CHECK(cudaMalloc((void**)&half_undistort_img_ptr_, img_attr_.double_depth_img_size_ / 4));
#endif

  // 初始化图像预处理参数
  img_attr_.model_input_width_ = 640;
  img_attr_.model_input_height_ = 640;
  img_attr_.resize_scale_ = std::min(static_cast<float>(img_attr_.model_input_width_)/ img_attr_.image_width_,
                                      static_cast<float>(img_attr_.model_input_height_) / img_attr_.image_height_);
  img_attr_.scaled_width_ = static_cast<uint32_t>(img_attr_.image_width_ * img_attr_.resize_scale_);
  img_attr_.scaled_height_ = static_cast<uint32_t>(img_attr_.image_height_ * img_attr_.resize_scale_);
  img_attr_.output_x_offset_ = (img_attr_.model_input_width_ - img_attr_.scaled_width_) / 2;
  img_attr_.output_y_offset_ = (img_attr_.model_input_height_ - img_attr_.scaled_height_) / 2;
  // check if display by rviz config yaml file
  auto all_cfg = rally::ConfigureManager::getInstance().getCfgNode();
  auto rviz_cfg = all_cfg["ros"]["rviz"];
  bool top_enable = rviz_cfg["enable"].as<bool>();
  bool left_enable = rviz_cfg["left_ac_vis"]["enable"].as<bool>();
  bool right_enable = rviz_cfg["right_ac_vis"]["enable"].as<bool>();
  display_ = top_enable && left_enable && right_enable;

  time_recorder_ptr_ = std::make_shared<TimeRecorder>(name());
  copy_vis_time_recorder_ptr_ = std::make_shared<TimeRecorder>(name()+"-copy_vis");
  AINFO << name() << ": finish init.";
}

void ObjectDetectionPreprocess::process(const Msg::Ptr &msg_ptr) {
  time_recorder_ptr_->tic();
  AINFO << name() << ": start process...";
  // 去畸变，并将去畸变图像指针保存到msg_ptr
#ifdef __aarch64__
  memcpy(distort_img_ptr_, msg_ptr->input_msg_ptr->image_map.at(rally::CameraEnum::left_ac_camera)->data.data, img_attr_.single_rgb_img_size_);
  memcpy(distort_img_ptr_ + img_attr_.single_rgb_img_size_, msg_ptr->input_msg_ptr->image_map.at(rally::CameraEnum::right_ac_camera)->data.data, img_attr_.single_rgb_img_size_);
#else
  CUDA_CHECK(cudaMemcpy(distort_img_ptr_, msg_ptr->input_msg_ptr->image_map.at(rally::CameraEnum::left_ac_camera)->data.data, img_attr_.single_rgb_img_size_, cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(distort_img_ptr_ + img_attr_.single_rgb_img_size_, msg_ptr->input_msg_ptr->image_map.at(rally::CameraEnum::right_ac_camera)->data.data, img_attr_.single_rgb_img_size_, cudaMemcpyHostToDevice));
#endif
  // 传入的图片就是去畸变的，所以这里直接拷贝过去
  // batched_image_universal_kernel_conversion(distort_img_ptr_,
  //                                           img_attr_.image_width_,
  //                                           img_attr_.image_width_ * 3,
  //                                           img_attr_.image_height_,
  //                                           img_attr_.batch_size_,
  //                                           img_attr_.image_width_,
  //                                           img_attr_.image_height_,
  //                                           0, 0, {0, 0, 0},
  //                                           undistort_img_ptr_,
  //                                           img_attr_.image_width_,
  //                                           img_attr_.image_width_ * 3,
  //                                           img_attr_.image_height_,
  //                                           DataType::Uint8,
  //                                           PixelLayout::NHWC_RGB,
  //                                           Interpolation::Bilinear,
  //                                           0.f, 0.f, 0.f, 1.f, 1.f, 1.f,
  //                                           remap_xy_,
  //                                           stream_
  //                                           );
  // msg_ptr->internal_result_ptr->undistorted_image_ptr = undistort_img_ptr_;
  undistort_img_ptr_ = distort_img_ptr_;
  msg_ptr->internal_result_ptr->undistorted_image_ptr = distort_img_ptr_;

  // resize到1/2，并将去畸变图像指针保存到msg_ptr
  batched_image_universal_kernel_conversion(undistort_img_ptr_,
                                            img_attr_.image_width_,
                                            img_attr_.image_width_ * 3,
                                            img_attr_.image_height_,
                                            img_attr_.batch_size_,
                                            img_attr_.half_width_,
                                            img_attr_.half_height_,
                                            0, 0, {0, 0, 0},
                                            half_undistort_img_ptr_,
                                            img_attr_.half_width_,
                                            img_attr_.half_width_ * 3,
                                            img_attr_.half_height_,
                                            DataType::Uint8,
                                            PixelLayout::NHWC_BGR,
                                            Interpolation::Bilinear,
                                            0.f, 0.f, 0.f, 1.f, 1.f, 1.f,
                                            nullptr,
                                            stream_
                                            );
  msg_ptr->internal_result_ptr->half_undistorted_image_ptr = half_undistort_img_ptr_;

  FillColor fillcolor;
  fillcolor.color[0] = 114;
  fillcolor.color[1] = 114;
  fillcolor.color[2] = 114;
  batched_image_universal_kernel_conversion(undistort_img_ptr_,
                                            img_attr_.image_width_,
                                            img_attr_.image_width_ * 3,
                                            img_attr_.image_height_,
                                            img_attr_.batch_size_,
                                            img_attr_.scaled_width_,
                                            img_attr_.scaled_height_,
                                            img_attr_.output_x_offset_,
                                            img_attr_.output_y_offset_,
                                            fillcolor,
                                            bindings_ptr_->od_bindings_device.image,
                                            img_attr_.model_input_width_,
                                            img_attr_.model_input_width_,
                                            img_attr_.model_input_height_,
                                            DataType::Float32,
                                            PixelLayout::NCHW_BGR,
                                            Interpolation::Bilinear,
                                            img_attr_.mean_rgb.at(0),
                                            img_attr_.mean_rgb.at(1),
                                            img_attr_.mean_rgb.at(2),
                                            img_attr_.std_rgb.at(0),
                                            img_attr_.std_rgb.at(1),
                                            img_attr_.std_rgb.at(2),
                                            nullptr,
                                            stream_
                                            );
  cudaStreamSynchronize(stream_);
  if(display_){
    // about 3 ms
    copy_vis_time_recorder_ptr_->tic();
    cv::Mat left_display(img_attr_.image_height_, img_attr_.image_width_,CV_8UC3);
    cv::Mat right_display(img_attr_.image_height_, img_attr_.image_width_,CV_8UC3);
    int batch_img_step = img_attr_.image_height_*img_attr_.image_width_*3;
#ifdef __aarch64__
    std::memcpy(left_display.data, undistort_img_ptr_, batch_img_step);
    std::memcpy(right_display.data, undistort_img_ptr_+batch_img_step, batch_img_step);
#else
    cudaMemcpy(left_display.data, undistort_img_ptr_, batch_img_step,cudaMemcpyDeviceToHost);
    cudaMemcpy(right_display.data, undistort_img_ptr_+batch_img_step, batch_img_step,cudaMemcpyDeviceToHost);
#endif
    left_display.copyTo(msg_ptr->internal_result_ptr->left_undistort_image);
    right_display.copyTo(msg_ptr->internal_result_ptr->right_undistort_image);
    copy_vis_time_recorder_ptr_->toc();
  }
  AINFO << name() << ": finish process.";
  time_recorder_ptr_->toc();
}

ObjectDetectionPreprocess::~ObjectDetectionPreprocess() {
#ifdef __aarch64__
  cudaFreeHost(remap_xy_);
  cudaFreeHost(distort_img_ptr_);
  cudaFreeHost(undistort_img_ptr_);
  cudaFreeHost(half_undistort_img_ptr_);
#else
  cudaFree(remap_xy_);
  cudaFree(distort_img_ptr_);
  cudaFree(undistort_img_ptr_);
  cudaFree(half_undistort_img_ptr_);
#endif
}

}
}
