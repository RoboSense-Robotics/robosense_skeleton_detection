//
// ROS1 版本的 Production - 订阅相机与雷达数据，向 Pipeline 提供数据
// 对应 ROS2 的 ros2_production
//

#ifndef POSE_DETECTION_ROS_PRODUCTION_H
#define POSE_DETECTION_ROS_PRODUCTION_H

#include <cmath>
#include <utility>

#include "motion_capture/interface/pipeline_interface.h"

#include <Eigen/src/Geometry/Quaternion.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

namespace robosense::motion_capture {

class RosProduction {
public:
  using Ptr = std::shared_ptr<RosProduction>;

  explicit RosProduction(ros::NodeHandle nh) : nh_(std::move(nh)) {}

  void init();

  void start();

  void regOpticalCallback(const std::function<void(const Msg::Ptr&)>& callback) {
    pipeline_ptr_->regOpticalCallback(callback);
  }

private:
  void leftCameraDebugCallback(const sensor_msgs::ImageConstPtr& data);
  void rightCameraDebugCallback(const sensor_msgs::ImageConstPtr& data);
  void leftLidarDebugCallback(const sensor_msgs::PointCloud2ConstPtr& data);
  void rightLidarDebugCallback(const sensor_msgs::PointCloud2ConstPtr& data);

  PointCloud::Ptr convertRsPointCloudToPointCloud(const sensor_msgs::PointCloud2ConstPtr& data);

private:
  ros::NodeHandle nh_;

  ros::Subscriber left_ac_camera_sub_;
  ros::Subscriber right_ac_camera_sub_;
  ros::Subscriber left_ac_lidar_debug_sub_;
  ros::Subscriber right_ac_lidar_debug_sub_;

  PipelineInterface::Ptr pipeline_ptr_;
  std::string frame_id_ = "base_link";

  TimeRecorder::Ptr left_camera_convert_recorder_;
  TimeRecorder::Ptr right_camera_convert_recorder_;
  TimeRecorder::Ptr left_lidar_convert_recorder_;
  TimeRecorder::Ptr right_lidar_convert_recorder_;
};

}  // namespace robosense::motion_capture

#endif  // POSE_DETECTION_ROS_PRODUCTION_H
