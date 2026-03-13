//
// ROS1 Production 实现 - 订阅相机与雷达，转换后送入 Pipeline
//

#include <spdlog/spdlog.h>
#include <cv_bridge/cv_bridge.h>

#include "production/ros_production.h"

namespace robosense::motion_capture {

void RosProduction::init() {
  spdlog::info("start init (ROS1)...");

  const auto& cfg_node = rally::ConfigureManager::getInstance().getCfgNode();
  const auto& ros_cfg_node = cfg_node["ros"];
  auto subscribe_node = ros_cfg_node["subscribe"];
  auto publish_node = ros_cfg_node["publish"];

  if (cfg_node["debug_mode"].as<bool>()) {
    // ROS1 使用 subscribe，TransportHints().tcpNoDelay() 近似 SensorDataQoS
    left_ac_camera_sub_ = nh_.subscribe(
        subscribe_node["left_ac_camera_offline_topic"].as<std::string>(),
        10,
        &RosProduction::leftCameraDebugCallback,
        this,
        ros::TransportHints().tcpNoDelay());

    right_ac_camera_sub_ = nh_.subscribe(
        subscribe_node["right_ac_camera_offline_topic"].as<std::string>(),
        10,
        &RosProduction::rightCameraDebugCallback,
        this,
        ros::TransportHints().tcpNoDelay());

    left_ac_lidar_debug_sub_ = nh_.subscribe(
        subscribe_node["left_ac_lidar_offline_topic"].as<std::string>(),
        10,
        &RosProduction::leftLidarDebugCallback,
        this,
        ros::TransportHints().tcpNoDelay());

    right_ac_lidar_debug_sub_ = nh_.subscribe(
        subscribe_node["right_ac_lidar_offline_topic"].as<std::string>(),
        10,
        &RosProduction::rightLidarDebugCallback,
        this,
        ros::TransportHints().tcpNoDelay());
  }

  auto glove_type = cfg_node["glove_type"].as<std::string>();
  spdlog::info("glove type: {}", glove_type);

  pipeline_ptr_ = std::make_shared<PipelineInterface>();
  pipeline_ptr_->init();

  left_camera_convert_recorder_ = std::make_shared<TimeRecorder>("left camera convert");
  right_camera_convert_recorder_ = std::make_shared<TimeRecorder>("right camera convert");
  left_lidar_convert_recorder_ = std::make_shared<TimeRecorder>("left lidar convert");
  right_lidar_convert_recorder_ = std::make_shared<TimeRecorder>("right lidar convert");

  spdlog::info("finish init (ROS1)...");
}

void RosProduction::leftCameraDebugCallback(const sensor_msgs::ImageConstPtr& data) {
  left_camera_convert_recorder_->tic();
  Image::Ptr image_ptr = std::make_shared<Image>();
  // ROS1: header.stamp 使用 sec + nsec
  image_ptr->timestamp =
      static_cast<uint64_t>(data->header.stamp.sec) * 1000000000ULL +
      static_cast<uint64_t>(data->header.stamp.nsec);
  cv::Mat rgb = cv_bridge::toCvCopy(data, "rgb8")->image;
  image_ptr->data = rgb.clone();

  Image::Ptr pre_image_ptr = std::make_shared<Image>();
  pre_image_ptr->timestamp = image_ptr->timestamp - 16500000U;
  pre_image_ptr->data = rgb.clone();

  left_camera_convert_recorder_->toc();
  pipeline_ptr_->addSensorData("left_ac_camera", pre_image_ptr);
  pipeline_ptr_->addSensorData("left_ac_camera", image_ptr);
}

void RosProduction::rightCameraDebugCallback(const sensor_msgs::ImageConstPtr& data) {
  right_camera_convert_recorder_->tic();
  Image::Ptr image_ptr = std::make_shared<Image>();
  image_ptr->timestamp =
      static_cast<uint64_t>(data->header.stamp.sec) * 1000000000ULL +
      static_cast<uint64_t>(data->header.stamp.nsec);
  cv::Mat rgb = cv_bridge::toCvCopy(data, "rgb8")->image;
  image_ptr->data = rgb.clone();

  Image::Ptr pre_image_ptr = std::make_shared<Image>();
  pre_image_ptr->timestamp = image_ptr->timestamp - 16500000U;
  pre_image_ptr->data = rgb.clone();

  right_camera_convert_recorder_->toc();
  pipeline_ptr_->addSensorData("right_ac_camera", pre_image_ptr);
  pipeline_ptr_->addSensorData("right_ac_camera", image_ptr);
}

void RosProduction::leftLidarDebugCallback(const sensor_msgs::PointCloud2ConstPtr& data) {
  left_lidar_convert_recorder_->tic();
  auto point_cloud_ptr = convertRsPointCloudToPointCloud(data);
  PointCloud::Ptr pre_point_cloud_ptr = std::make_shared<PointCloud>();
  pre_point_cloud_ptr->timestamp = point_cloud_ptr->timestamp - 33000000U;
  *(pre_point_cloud_ptr->data) = *(point_cloud_ptr->data);
  PointCloud::Ptr next_point_cloud_ptr = std::make_shared<PointCloud>();
  next_point_cloud_ptr->timestamp = point_cloud_ptr->timestamp + 33000000U;
  *(next_point_cloud_ptr->data) = *(point_cloud_ptr->data);
  left_lidar_convert_recorder_->toc();
  pipeline_ptr_->addSensorData("left_ac_lidar", pre_point_cloud_ptr);
  pipeline_ptr_->addSensorData("left_ac_lidar", point_cloud_ptr);
  pipeline_ptr_->addSensorData("left_ac_lidar", next_point_cloud_ptr);
}

void RosProduction::rightLidarDebugCallback(const sensor_msgs::PointCloud2ConstPtr& data) {
  right_lidar_convert_recorder_->tic();
  auto point_cloud_ptr = convertRsPointCloudToPointCloud(data);
  PointCloud::Ptr pre_point_cloud_ptr = std::make_shared<PointCloud>();
  pre_point_cloud_ptr->timestamp = point_cloud_ptr->timestamp - 33000000U;
  *(pre_point_cloud_ptr->data) = *(point_cloud_ptr->data);
  PointCloud::Ptr next_point_cloud_ptr = std::make_shared<PointCloud>();
  next_point_cloud_ptr->timestamp = point_cloud_ptr->timestamp + 33000000U;
  *(next_point_cloud_ptr->data) = *(point_cloud_ptr->data);
  right_lidar_convert_recorder_->toc();
  pipeline_ptr_->addSensorData("right_ac_lidar", pre_point_cloud_ptr);
  pipeline_ptr_->addSensorData("right_ac_lidar", point_cloud_ptr);
  pipeline_ptr_->addSensorData("right_ac_lidar", next_point_cloud_ptr);
}

PointCloud::Ptr RosProduction::convertRsPointCloudToPointCloud(
    const sensor_msgs::PointCloud2ConstPtr& data) {
  // ROS1 中 pcl::fromROSMsg 可直接使用 sensor_msgs::PointCloud2
  PointCloud::Ptr point_cloud_ptr = std::make_shared<PointCloud>();
  point_cloud_ptr->timestamp =
      static_cast<uint64_t>(data->header.stamp.sec) * 1000000000ULL +
      static_cast<uint64_t>(data->header.stamp.nsec);
  pcl::fromROSMsg(*data, *point_cloud_ptr->data);
  return point_cloud_ptr;
}

void RosProduction::start() {
  pipeline_ptr_->start();
}

}  // namespace robosense::motion_capture
