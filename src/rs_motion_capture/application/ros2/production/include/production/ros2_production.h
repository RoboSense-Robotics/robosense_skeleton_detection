//
// Created by sti on 2025/6/13.
//

#ifndef POSE_DETECTION_ROS_PRODUCTION_H
#define POSE_DETECTION_ROS_PRODUCTION_H

#include "motion_capture/interface/pipeline_interface.h"
#include <Eigen/src/Geometry/Quaternion.h>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
// #include "robosense_msgs/msg/rs_image.hpp"
// #include "robosense_msgs/msg/rs_point_cloud.hpp"
// #include "robosense_msgs/msg/rs_compressed_image.hpp"
#include "std_msgs/msg/string.hpp"
#include <cv_bridge/cv_bridge.h>

namespace robosense {
namespace motion_capture {

class Ros2Production {
public:
  using Ptr = std::shared_ptr<Ros2Production>;

  Ros2Production(const rclcpp::Node::SharedPtr& node_ptr) : node_ptr_(node_ptr) {
  };

  void init();

  void start();

  void regOpticalCallback(const std::function<void(const Msg::Ptr&)>& callback) {
    pipeline_ptr_->regOpticalCallback(callback);
  }

  void regInertialCallback(const std::function<void(const Msg::Ptr&)>& callback) {
    pipeline_ptr_->regInertialCallback(callback);
  }

  std::string name() const {
    return "Ros2Production";
  }

private:
  void leftCameraDebugCallback(const sensor_msgs::msg::Image::SharedPtr data);

  void rightCameraDebugCallback(const sensor_msgs::msg::Image::SharedPtr data);
  
  void leftLidarDebugCallback(const sensor_msgs::msg::PointCloud2::SharedPtr data);
  
  void rightLidarDebugCallback(const sensor_msgs::msg::PointCloud2::SharedPtr data);

  void noitonHandPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr data);

  void manusHandPoseCallback(const sensor_msgs::msg::JointState::SharedPtr data);

  template <typename PointCloudType>
  PointCloud::Ptr convertRsPointCloudToPointCloud(const PointCloudType data);

  void publishFingerDistance(const Msg::Ptr& msg_ptr);

  void publishPose(const Msg::Ptr& msg_ptr);

  void publishPose2d(const Msg::Ptr& msg_ptr);

  void publishHandEuler(const Msg::Ptr& msg_ptr);

  void publishCheckHci(const Msg::Ptr& msg_ptr);

  void publishCalibHci(const Msg::Ptr& msg_ptr);

private:
  rclcpp::Node::SharedPtr node_ptr_;

  rclcpp::SubscriptionBase::SharedPtr left_ac_camera_sub_;
  rclcpp::SubscriptionBase::SharedPtr right_ac_camera_sub_;
  // rclcpp::Subscription<robosense_msgs::msg::RsPointCloud>::SharedPtr left_ac_lidar_sub_;
  // rclcpp::Subscription<robosense_msgs::msg::RsPointCloud>::SharedPtr right_ac_lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_ac_lidar_debug_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_ac_lidar_debug_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr noiton_hand_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  manus_hand_pose_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr left_finger_distance_pub_;    // 左手指距离
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr right_finger_distance_pub_;   // 右手指距离
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_3d_pub_;                 // 关键点和末端位姿
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_2d_pub_;                 // 网络2d点
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr hand_euler_pub_;               // 手指关节角
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_HciPublisher_;                       // 语音播放节点
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_ProgressPublisher_;                       // 语音播放节点

  PipelineInterface::Ptr pipeline_ptr_;
  std::string frame_id_ = "base_link";

  TimeRecorder::Ptr left_camera_convert_recorder_;
  TimeRecorder::Ptr right_camera_convert_recorder_;
  TimeRecorder::Ptr left_lidar_convert_recorder_;
  TimeRecorder::Ptr right_lidar_convert_recorder_;

  /*const float PI = 3.1415926;*/
  // 右手绕z轴正转120
  // 左手绕z轴-转120
  Eigen::Quaternionf left_rot{static_cast<float>(std::cos(-M_PI/3)), 0.f, 0.f, static_cast<float>(std::sin(-M_PI/3))};
  Eigen::Quaternionf right_rot{static_cast<float>(std::cos(M_PI/3)), 0.f, 0.f, static_cast<float>(std::sin(M_PI/3))};

  // 语音相关
  bool pub_calib_start_ = false;
  bool pub_calib_end_ = false;
  bool pub_check_start_ = false;
  bool pub_check_end_ = false;
};

}
}

#endif //POSE_DETECTION_ROS_PRODUCTION_H
