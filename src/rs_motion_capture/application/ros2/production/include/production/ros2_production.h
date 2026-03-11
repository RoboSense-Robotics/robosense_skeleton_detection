//
// Created by sti on 2025/6/13.
//

#ifndef POSE_DETECTION_ROS_PRODUCTION_H
#define POSE_DETECTION_ROS_PRODUCTION_H

#include <cmath>
#include <utility>

#include "motion_capture/interface/pipeline_interface.h"

#include <Eigen/src/Geometry/Quaternion.h>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>

namespace robosense::motion_capture {

class Ros2Production {
public:
  using Ptr = std::shared_ptr<Ros2Production>;

  explicit Ros2Production(rclcpp::Node::SharedPtr node_ptr) : node_ptr_(std::move(node_ptr))
  {};

  void init();

  void start();

  void regOpticalCallback(const std::function<void(const Msg::Ptr&)>& callback) {
    pipeline_ptr_->regOpticalCallback(callback);
  }

private:
  void leftCameraDebugCallback(sensor_msgs::msg::Image::SharedPtr data);

  void rightCameraDebugCallback(sensor_msgs::msg::Image::SharedPtr data);
  
  void leftLidarDebugCallback(sensor_msgs::msg::PointCloud2::SharedPtr data);
  
  void rightLidarDebugCallback(sensor_msgs::msg::PointCloud2::SharedPtr data);

  template <typename PointCloudType>
  PointCloud::Ptr convertRsPointCloudToPointCloud(PointCloudType data);

private:
  rclcpp::Node::SharedPtr node_ptr_;

  rclcpp::SubscriptionBase::SharedPtr left_ac_camera_sub_;
  rclcpp::SubscriptionBase::SharedPtr right_ac_camera_sub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_ac_lidar_debug_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_ac_lidar_debug_sub_;

  PipelineInterface::Ptr pipeline_ptr_;
  std::string frame_id_ = "base_link";

  TimeRecorder::Ptr left_camera_convert_recorder_;
  TimeRecorder::Ptr right_camera_convert_recorder_;
  TimeRecorder::Ptr left_lidar_convert_recorder_;
  TimeRecorder::Ptr right_lidar_convert_recorder_;
};

}


#endif //POSE_DETECTION_ROS_PRODUCTION_H
