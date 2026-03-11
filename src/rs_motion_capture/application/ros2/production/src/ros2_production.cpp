//
// Created by sti on 2025/6/13.
//

#include <spdlog/spdlog.h>
#include <cv_bridge/cv_bridge.h>

#include "production/ros2_production.h"

namespace robosense::motion_capture {

void Ros2Production::init()
{
  spdlog::info("start init...");

  const auto& cfg_node = rally::ConfigureManager::getInstance().getCfgNode();
  const auto& ros_cfg_node = cfg_node["ros"];
  auto subscribe_node = ros_cfg_node["subscribe"];
  auto publish_node = ros_cfg_node["publish"];

  if (cfg_node["debug_mode"].as<bool>())
  {
    left_ac_camera_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::Image>(
        subscribe_node["left_ac_camera_offline_topic"].as<std::string>(),
        rclcpp::SensorDataQoS(),
        std::bind(&Ros2Production::leftCameraDebugCallback, this, std::placeholders::_1));

    right_ac_camera_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::Image>(
        subscribe_node["right_ac_camera_offline_topic"].as<std::string>(),
        rclcpp::SensorDataQoS(),
        std::bind(&Ros2Production::rightCameraDebugCallback, this, std::placeholders::_1));

    left_ac_lidar_debug_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::PointCloud2>(
        subscribe_node["left_ac_lidar_offline_topic"].as<std::string>(),
        rclcpp::SensorDataQoS(),
        std::bind(&Ros2Production::leftLidarDebugCallback, this, std::placeholders::_1));

    right_ac_lidar_debug_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::PointCloud2>(
        subscribe_node["right_ac_lidar_offline_topic"].as<std::string>(),
        rclcpp::SensorDataQoS(),
        std::bind(&Ros2Production::rightLidarDebugCallback, this, std::placeholders::_1));
  }

  auto glove_type = cfg_node["glove_type"].as<std::string>();
  spdlog::info("glove type: {}", glove_type);

  pipeline_ptr_ = std::make_shared<PipelineInterface>();
  pipeline_ptr_->init();

  left_camera_convert_recorder_ = std::make_shared<TimeRecorder>("left camera convert");
  right_camera_convert_recorder_ = std::make_shared<TimeRecorder>("right camera convert");
  left_lidar_convert_recorder_ = std::make_shared<TimeRecorder>("left lidar convert");
  right_lidar_convert_recorder_ = std::make_shared<TimeRecorder>("right lidar convert");

  spdlog::info("finish init...");
}

void Ros2Production::leftCameraDebugCallback(sensor_msgs::msg::Image::SharedPtr data)
{
    left_camera_convert_recorder_->tic();
    Image::Ptr image_ptr = std::make_shared<Image>();
    image_ptr->timestamp =
        static_cast<uint64_t>(data->header.stamp.sec) * 1000000000ULL +
        static_cast<uint64_t>(data->header.stamp.nanosec);
    cv::Mat rgb = cv_bridge::toCvCopy(data, "rgb8")->image;
    image_ptr->data = rgb.clone();

    Image::Ptr pre_image_ptr = std::make_shared<Image>();
    Image::Ptr next_image_ptr = std::make_shared<Image>();
    pre_image_ptr->timestamp = image_ptr->timestamp - 16500000U;
    pre_image_ptr->data = rgb.clone();

    left_camera_convert_recorder_->toc();
    pipeline_ptr_->addSensorData("left_ac_camera", pre_image_ptr);
    pipeline_ptr_->addSensorData("left_ac_camera", image_ptr);

}

void Ros2Production::rightCameraDebugCallback(sensor_msgs::msg::Image::SharedPtr data)
{
    right_camera_convert_recorder_->tic();
    Image::Ptr image_ptr = std::make_shared<Image>();
    image_ptr->timestamp =
        static_cast<uint64_t>(data->header.stamp.sec) * 1000000000ULL +
        static_cast<uint64_t>(data->header.stamp.nanosec);
    cv::Mat rgb = cv_bridge::toCvCopy(data, "rgb8")->image;
    image_ptr->data = rgb.clone();

    Image::Ptr pre_image_ptr = std::make_shared<Image>();
    Image::Ptr next_image_ptr = std::make_shared<Image>();
    pre_image_ptr->timestamp = image_ptr->timestamp - 16500000U;
    pre_image_ptr->data = rgb.clone();

    right_camera_convert_recorder_->toc();
    pipeline_ptr_->addSensorData("right_ac_camera", pre_image_ptr);
    pipeline_ptr_->addSensorData("right_ac_camera", image_ptr);

}

void Ros2Production::leftLidarDebugCallback(sensor_msgs::msg::PointCloud2::SharedPtr data)
{
  left_lidar_convert_recorder_->tic();
  auto point_cloud_ptr = convertRsPointCloudToPointCloud(std::move(data));
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

void Ros2Production::rightLidarDebugCallback(sensor_msgs::msg::PointCloud2::SharedPtr data)
{
  right_lidar_convert_recorder_->tic();
  auto point_cloud_ptr = convertRsPointCloudToPointCloud(std::move(data));
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

template <typename PointCloudType>
PointCloud::Ptr Ros2Production::convertRsPointCloudToPointCloud(PointCloudType data)
{
  auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud_msg->header.stamp = data->header.stamp;
  cloud_msg->height = data->height;
  cloud_msg->width = data->width;
  cloud_msg->is_bigendian = data->is_bigendian;
  cloud_msg->point_step = data->point_step;
  cloud_msg->row_step = data->row_step;
  cloud_msg->is_dense = data->is_dense;
  cloud_msg->fields.resize(data->fields.size());
  for (size_t i = 0; i < data->fields.size(); i++)
  {
    cloud_msg->fields[i].name = std::string(reinterpret_cast<const char*>(data->fields[i].name.data()),
                                            strnlen(reinterpret_cast<const char*>(data->fields[i].name.data()),
                                                    data->fields[i].name.size()));
    cloud_msg->fields[i].offset = data->fields[i].offset;
    cloud_msg->fields[i].datatype = data->fields[i].datatype;
    cloud_msg->fields[i].count = data->fields[i].count;
  }
  size_t data_size = data->row_step * data->height;
  cloud_msg->data.resize(data_size);
  memcpy(cloud_msg->data.data(), data->data.data(), data_size);

  PointCloud::Ptr point_cloud_ptr = std::make_shared<PointCloud>();
  point_cloud_ptr->timestamp = data->header.stamp.sec * 1e9 + data->header.stamp.nanosec;
  pcl::fromROSMsg(*cloud_msg, *point_cloud_ptr->data);
  return point_cloud_ptr;
}

void Ros2Production::start()
{
  pipeline_ptr_->start();
}
}
