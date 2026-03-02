//
// Created by sti on 2025/6/6.
//

#ifndef MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_POSE_DETECTION_POSTPROCESS_H
#define MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_POSE_DETECTION_POSTPROCESS_H

#include <cuda_runtime.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "motion_capture/common/message.h"
#include "motion_capture/strategy/optical/two_stage/common/bindings.h"
#include "motion_capture/data_structure/point.h"
#include "motion_capture/algorithm/geometry/kalman.h"

namespace robosense {
namespace motion_capture {

class PoseDetectionPostprocess {
public:
  using Ptr = std::shared_ptr<PoseDetectionPostprocess>;

  PoseDetectionPostprocess(const Bindings::Ptr &bindings_ptr, cudaStream_t stream)
          : bindings_ptr_(bindings_ptr), stream_(stream) {}

  void init(const YAML::Node& cfg_node);

  void process(const Msg::Ptr &msg_ptr);

  std::string name() const { return "PoseDetectionPostprocess"; }

private:
  void fusionOptRes();


  void processSingleImpl(const Msg::Ptr& msg_ptr, rally::CameraEnum camera_enum);

  std::vector<Point2f> getModelOutputKeyPts(const Msg::Ptr& msg_ptr, rally::CameraEnum camera_enum);

  void getAllKeyPts(const Msg::Ptr& msg_ptr,
                    const std::vector<Point2f>& res,
                    rally::CameraEnum camera_enum);

  std::vector<Point2f> getImageCoordKeyPts(const Msg::Ptr& msg_ptr,
                                           const std::vector<Point2f>& res,
                                           rally::CameraEnum camera_enum);

  std::vector<Point3f> getWorldCoordKeyPts(const Msg::Ptr &msg_ptr,
                                           const std::vector<Point2f> &img_arm_key_pts,
                                           const rally::CameraEnum &camera_enum);


  std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, std::pair<Eigen::Vector3f, Eigen::Quaternionf>>
  calculateEndEffectorPose(const Msg::Ptr &msg_ptr, const std::vector<Point3f> &pose);

  Eigen::Quaternionf computeHandRotationFromBone(
          const Eigen::Vector3f& p1,
          const Eigen::Vector3f& p2,
          const Eigen::Vector3f& p3);

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
  float score_threshold_ = 0.3f;
  float simcc_ratio_ = 0.5f;
  uint32_t extend_width_ = 576;
  uint32_t extend_height_ = 768;
  uint32_t num_points_ = 26;

  //
  Bindings::Ptr bindings_ptr_;
  cudaStream_t stream_;

  std::vector<int> all_key_pts_idx_ = {6, 8, 5, 7, 12, 11};  // 躯干索引
  std::vector<int> hip_key_pts_idx_ = {12, 11}; // 左右中点索引
  std::vector<int> shouder_key_pts_idx_ = {6, 5};  // 肩膀关键点索引
  std::vector<int> arm_key_pts_idx_ = {6, 8, 10 ,5 ,7, 9};  // 手臂关键点索引
  std::vector<int> left_hand_key_pts_idx_ = {10};  // 左手关键点索引
  std::vector<int> right_hand_key_pts_idx_ = {9};  // 右手关键点索引
  std::vector<int> lower_body_key_pts_idx_ = {12, 14, 16, 11, 13, 15}; // 下半身关键点索引


  std::map<rally::CameraEnum, std::vector<KalmanFilter1D>> filters_1d_map_;
  std::map<rally::CameraEnum, std::vector<KalmanFilter2D>> filters_2d_map_;

  bool use_aruco_ = true; // 是否使用aruco
  std::map<rally::CameraEnum, uint8_t> last_triangle_pt_valid_map_;
  std::map<rally::CameraEnum, std::vector<cv::Point2f>> last_triangle_pt_map_; // 上一帧的三角形点
  int filter_nums_ = 15;



  std::map<rally::CameraEnum, uint8_t> last_aruco_pt_valid_map_;
  std::map<rally::CameraEnum, Point2f> last_aruco_pt_map_; // 上一帧的三角形点

  float ema_beta_ = 0.8;
  std::map<rally::CameraEnum, std::vector<float>> ema_key_pts_depth_map_;

  std::string glove_type_ = "noiton"; // 手套类型

private:
  TimeRecorder::Ptr time_recorder_ptr_;



};

}
}

#endif //MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_POSE_DETECTION_POSTPROCESS_H
