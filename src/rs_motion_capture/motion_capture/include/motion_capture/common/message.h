//
// Created by sti on 2025/6/6.
//

#ifndef POSE_DETECTION_MESSAGE_H
#define POSE_DETECTION_MESSAGE_H

#include "rally/common/common.h"
#include "rally/utils/utils.h"
#include "motion_capture/data_structure/bbox.h"
#include "motion_capture/data_structure/point.h"
#include "motion_capture/data_structure/image.h"
#include "motion_capture/data_structure/point_cloud.h"
#include "motion_capture/data_structure/joint_pose.h"
#include "motion_capture/utils/performance/time_recorder.h"

namespace robosense {
namespace motion_capture {

/// @brief 输入消息类，包含相机图像、激光雷达点云和手部关节姿态
class InputMsg {
public:
  using Ptr = std::shared_ptr<InputMsg>;
  std::map<rally::CameraEnum, Image::Ptr> image_map;
  std::map<rally::LidarEnum, PointCloud::Ptr> lidar_map;
  JointPose::Ptr hand_joints_pose;
  JointPose2::Ptr hand_joints_pose2;
  JointPose2::Ptr left_infrared_tracker = nullptr;
  JointPose2::Ptr right_infrared_tracker = nullptr;
};

/// @brief 内部处理结果类，用于保存中间结果，用于在模块间传递、可视化、debug
class InternalResult {
public:
  using Ptr = std::shared_ptr<InternalResult>;

  InternalResult() {
    qr_code_is_detected_map.clear();
    center_map.clear();
    is_person_detected_map.clear();
    detected_person_bbox_map.clear();
    pose_model_input_bbox_map.clear();
    image_all_pose_points_map.clear();
    image_projected_pc_map.clear();
    image_projected_pc_depth_map.clear();
    image_arm_key_points_map.clear();
    camera_arm_key_points_map.clear();
    world_arm_key_points_map.clear();
    world_end_pose_map.clear();
    left_finger_distance_map[rally::CameraEnum::left_ac_camera] = -1.f;
    left_finger_distance_map[rally::CameraEnum::right_ac_camera] = -1.f;
    right_finger_distance_map[rally::CameraEnum::left_ac_camera] = -1.f;
    right_finger_distance_map[rally::CameraEnum::right_ac_camera] = -1.f;
    fusion_optimized_world_arm_key_points.clear();
    calib_finish_map[rally::CameraEnum::left_ac_camera] = false;
    calib_finish_map[rally::CameraEnum::right_ac_camera] = false;
    check_finish = false;
    check_progress = 0.0;
    calib_progress = 0.0;
    calib_info = "";
  }



  uint8_t* undistorted_image_ptr = nullptr;
  uint8_t* half_undistorted_image_ptr = nullptr;

  cv::Mat left_undistort_image;
  cv::Mat right_undistort_image;

  // 二维码中心在原始图像中的位置
  std::map<rally::CameraEnum, uint8_t> qr_code_is_detected_map;
  std::map<rally::CameraEnum, Point2f> center_map;

  // 是否检测到人
  std::map<rally::CameraEnum, uint8_t> is_person_detected_map;
  std::map<rally::CameraEnum, BBox<float>> detected_person_bbox_map;

  // 检测到的人的信息（pose检测网络输入的box信息）
  std::map<rally::CameraEnum, BBox<float>> pose_model_input_bbox_map;

  // 图像下检测到的所有pose点（133）
  std::map<rally::CameraEnum, std::vector<Point2f>> image_all_pose_points_map;

  // 点云投影到图像坐标系下的点坐标和深度
  std::map<rally::CameraEnum, pcl::PointCloud<pcl::PointXY>::Ptr> image_projected_pc_map;
  std::map<rally::CameraEnum, std::shared_ptr<std::vector<float>>> image_projected_pc_depth_map;

  // 图像坐标系下的人体关键点
  std::map<rally::CameraEnum, std::vector<Point2f>> image_arm_key_points_map;

  // 相机坐标系下的人体关键点
  std::map<rally::CameraEnum, std::vector<Point3f>> camera_arm_key_points_map;

  // 世界坐标系下的人体关键点
  std::map<rally::CameraEnum, std::vector<Point3f>> world_arm_key_points_map;

  // 世界坐标系下的末端位置和朝向
  std::map<rally::CameraEnum, std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, std::pair<Eigen::Vector3f, Eigen::Quaternionf>>> world_end_pose_map;

  // 图像上检测到的手指距离
  std::map<rally::CameraEnum, float> left_finger_distance_map;
  std::map<rally::CameraEnum, float> right_finger_distance_map;

  // 融合优化后的世界坐标系下的人体关键点
  std::vector<Point3f> fusion_optimized_world_arm_key_points;

  // 融合优化后的世界坐标系下的末端位置和朝向
  std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, std::pair<Eigen::Vector3f, Eigen::Quaternionf>> fusion_optimized_world_end_pose;

  std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, std::pair<Eigen::Vector3f, Eigen::Quaternionf>> glove_passthrough_pose;

  // 标定是否完成
  std::map<rally::CameraEnum, bool> calib_finish_map;

  // 检查是否完成
  bool check_finish;

  // 检查进度
  float check_progress;
  // 骨骼标定进度
  float calib_progress;
  // 标定提示信息
  std::string calib_info;
};

/// @brief 输出消息类，包含最终输出的手指距离、人体关键点和末端位置朝向，用于向下游发布
class OutputMsg {
public:
  using Ptr = std::shared_ptr<OutputMsg>;
  // 左右手指距离
  float left_finger_distance = 0.0f;
  float right_finger_distance = 0.0f;
  // 人本体坐标系下的关键点
  std::vector<Point3f> arm_key_points;
  // 人本体坐标系下的末端位置和朝向
  std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, std::pair<Eigen::Vector3f, Eigen::Quaternionf>> end_pose;
  // 左右手指自由度弯曲角度
  std::vector<float> finger_angles;
};

/// @brief 消息类
class Msg {
public:
  using Ptr = std::shared_ptr<Msg>;

  Msg() {
    input_msg_ptr = std::make_shared<InputMsg>();
    internal_result_ptr = std::make_shared<InternalResult>();
    output_msg_ptr = std::make_shared<OutputMsg>();
  }
  bool valid_flag = true;
  uint32_t seq;
  uint64_t timestamp;
  InputMsg::Ptr input_msg_ptr;
  InternalResult::Ptr internal_result_ptr;
  OutputMsg::Ptr output_msg_ptr;

  bool use_glove;
};

}
}

#endif //POSE_DETECTION_MESSAGE_H
