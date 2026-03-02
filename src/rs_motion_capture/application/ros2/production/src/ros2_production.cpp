//
// Created by sti on 2025/6/13.
//

#include "production/ros2_production.h"
#include <Eigen/src/Geometry/Quaternion.h>
#include <rs_log/common/log.h>
#include <cv_bridge/cv_bridge.h>
namespace robosense {
namespace motion_capture {

void Ros2Production::init() {
  AINFO << name() << ": start init...";
  const auto& cfg_node = rally::ConfigureManager::getInstance().getCfgNode();
  const auto& ros_cfg_node = cfg_node["ros"];
  auto subscribe_node = ros_cfg_node["subscribe"];
  auto publish_node = ros_cfg_node["publish"];

  float left_z_rot = cfg_node["left_hand_z_rot"].as<float>();
  float right_z_rot = cfg_node["right_hand_z_rot"].as<float>();
  left_rot.w() = std::cos(left_z_rot / 180 * M_PI / 2);
  left_rot.x() = 0;
  left_rot.y() = 0;
  left_rot.z() = std::sin(left_z_rot/180*M_PI / 2);

  right_rot.w() = std::cos(right_z_rot / 180 * M_PI / 2);
  right_rot.x() = 0;
  right_rot.y() = 0;
  right_rot.z() = std::sin(right_z_rot/180*M_PI / 2);

  AINFO << "left rot wxyz: "<< left_rot.w() <<" "<<left_rot.x()<<" "<<left_rot.y()<<" "<<left_rot.z();
  AINFO << "right rot wxyz: "<< right_rot.w() <<" "<<right_rot.x()<<" "<<right_rot.y()<<" "<<right_rot.z();

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

  std::string glove_type = cfg_node["glove_type"].as<std::string>();
  AINFO << glove_type << std::endl;
  if ("noiton" == glove_type) {
    noiton_hand_pose_sub_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseArray>(
        subscribe_node["noiton_glove_topic"].as<std::string>(),
        rclcpp::SensorDataQoS(),
        std::bind(&Ros2Production::noitonHandPoseCallback, this, std::placeholders::_1));
  } else if ("manus" == glove_type) {
    manus_hand_pose_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::JointState>(
      subscribe_node["manus_glove_topic"].as<std::string>(),
      rclcpp::SensorDataQoS(),
      std::bind(&Ros2Production::manusHandPoseCallback, this, std::placeholders::_1));
  } else {
    RERROR << "glove_type not found";
  }

  pose_2d_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseArray>(
          publish_node["pose_2d_topic"].as<std::string>(), 10);

  pose_3d_pub_ = node_ptr_->create_publisher<geometry_msgs::msg::PoseArray>(
          publish_node["pose_3d_topic"].as<std::string>(), 10);

  hand_euler_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::JointState>(
          publish_node["hand_euler_topic"].as<std::string>(), 10);

  left_finger_distance_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::Temperature>(
          publish_node["left_finger_distance_topic"].as<std::string>(), 10);

  right_finger_distance_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::Temperature>(
          publish_node["right_finger_distance_topic"].as<std::string>(), 10);

  // 语音交互
  m_HciPublisher_ = 
          node_ptr_->create_publisher<std_msgs::msg::String>("/robot_control/hci_status", 10);
  // 进度条信息
  m_ProgressPublisher_ = 
          node_ptr_->create_publisher<std_msgs::msg::String>("/mqtt/calib_progress", 10);

  pipeline_ptr_ = std::make_shared<PipelineInterface>();
  pipeline_ptr_->init();

  if (cfg_node["check_mode"].as<bool>()) {
    pipeline_ptr_->regOpticalCallback(
      std::bind(&Ros2Production::publishCheckHci, this, std::placeholders::_1));
  } else if (cfg_node["calib_mode"].as<bool>()) {
    pipeline_ptr_->regOpticalCallback(
      std::bind(&Ros2Production::publishCalibHci, this, std::placeholders::_1));
  }

  if (!cfg_node["calib_mode"].as<bool>()) {
    // 只有在非标定模式下才发布结果
    pipeline_ptr_->regOpticalCallback(
            std::bind(&Ros2Production::publishPose, this, std::placeholders::_1));
    pipeline_ptr_->regOpticalCallback(
            std::bind(&Ros2Production::publishPose2d, this, std::placeholders::_1));
	if (glove_type == "noiton") {
    	pipeline_ptr_->regInertialCallback(
            std::bind(&Ros2Production::publishHandEuler, this, std::placeholders::_1));
    	auto pipeline_strategy = cfg_node["pipeline"]["strategy"].as<std::string>();
    	if (pipeline_strategy == "HybridOpticalInertialMotionCapture") {
      		// 光惯组合模式下，手指开合由惯性动捕发布
      		pipeline_ptr_->regInertialCallback(
              std::bind(&Ros2Production::publishFingerDistance, this, std::placeholders::_1));
    	} else if (pipeline_strategy == "OpticalMotionCapture") {
      		// 光学动捕模式下，手指开合由光学动捕发布
      		pipeline_ptr_->regOpticalCallback(
              std::bind(&Ros2Production::publishFingerDistance, this, std::placeholders::_1));
    	} else {
      		RWARN << "Unknown pipeline strategy: " + pipeline_strategy + ", not publish finger distance.";
    	}
  	}
  }
  left_camera_convert_recorder_ = std::make_shared<TimeRecorder>("left camera convert");
  right_camera_convert_recorder_ = std::make_shared<TimeRecorder>("right camera convert");
  left_lidar_convert_recorder_ = std::make_shared<TimeRecorder>("left lidar convert");
  right_lidar_convert_recorder_ = std::make_shared<TimeRecorder>("right lidar convert");
  AINFO << name() << ": finish init.";
}

void Ros2Production::leftCameraDebugCallback(
    const sensor_msgs::msg::Image::SharedPtr data)
{
    left_camera_convert_recorder_->tic();
    Image::Ptr image_ptr = std::make_shared<Image>();
    image_ptr->timestamp =
        static_cast<uint64_t>(data->header.stamp.sec) * 1e9 +
        static_cast<uint64_t>(data->header.stamp.nanosec);
    cv::Mat rgb = cv_bridge::toCvCopy(data, "rgb8")->image;
    image_ptr->data = rgb.clone();

    Image::Ptr pre_image_ptr = std::make_shared<Image>();
    Image::Ptr next_image_ptr = std::make_shared<Image>();
    pre_image_ptr->timestamp = image_ptr->timestamp - 16500000U;
    pre_image_ptr->data = rgb.clone();
    /*next_image_ptr->timestamp = image_ptr->timestamp + 20000000U;*/
    /*next_image_ptr->data = rgb.clone();*/
    left_camera_convert_recorder_->toc();
    pipeline_ptr_->addSensorData("left_ac_camera", pre_image_ptr);
    pipeline_ptr_->addSensorData("left_ac_camera", image_ptr);
    /*pipeline_ptr_->addSensorData("left_ac_camera", next_image_ptr);*/
}

void Ros2Production::rightCameraDebugCallback(
    const sensor_msgs::msg::Image::SharedPtr data)
{
    right_camera_convert_recorder_->tic();
    Image::Ptr image_ptr = std::make_shared<Image>();
    image_ptr->timestamp =
        static_cast<uint64_t>(data->header.stamp.sec) * 1e9 +
        static_cast<uint64_t>(data->header.stamp.nanosec);
    cv::Mat rgb = cv_bridge::toCvCopy(data, "rgb8")->image;
    image_ptr->data = rgb.clone();

    Image::Ptr pre_image_ptr = std::make_shared<Image>();
    Image::Ptr next_image_ptr = std::make_shared<Image>();
    pre_image_ptr->timestamp = image_ptr->timestamp - 16500000U;
    pre_image_ptr->data = rgb.clone();
    /*next_image_ptr->timestamp = image_ptr->timestamp + 20000000U;*/
    /*next_image_ptr->data = rgb.clone();*/
    right_camera_convert_recorder_->toc();
    pipeline_ptr_->addSensorData("right_ac_camera", pre_image_ptr);
    pipeline_ptr_->addSensorData("right_ac_camera", image_ptr);
    /*pipeline_ptr_->addSensorData("right_ac_camera", next_image_ptr);*/
}

void Ros2Production::leftLidarDebugCallback(const sensor_msgs::msg::PointCloud2::SharedPtr data) {
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

void Ros2Production::rightLidarDebugCallback(const sensor_msgs::msg::PointCloud2::SharedPtr data) {
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

void Ros2Production::noitonHandPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr data) {
  JointPose::Ptr joint_pose_ptr = std::make_shared<JointPose>();
  joint_pose_ptr->timestamp = data->header.stamp.sec * 1e9 + data->header.stamp.nanosec;
  joint_pose_ptr->data->resize(data->poses.size());
  for (size_t i = 0; i < data->poses.size(); ++i) {
    joint_pose_ptr->data->at(i) = Eigen::Quaternionf(data->poses[i].orientation.w,
                                                     data->poses[i].orientation.x,
                                                     data->poses[i].orientation.y,
                                                     data->poses[i].orientation.z);
  }
  pipeline_ptr_->addSensorData("hand_pose", joint_pose_ptr);
}

void Ros2Production::manusHandPoseCallback(const sensor_msgs::msg::JointState::SharedPtr data) {
  JointPose2::Ptr joint_pose_ptr = std::make_shared<JointPose2>();
  joint_pose_ptr->timestamp = data->header.stamp.sec * 1e9 + data->header.stamp.nanosec;
  joint_pose_ptr->data->resize(data->position.size());
  for (size_t i = 0; i < data->position.size(); ++i) {
    joint_pose_ptr->data->at(i) = data->position[i];
  }
  pipeline_ptr_->addSensorData("hand_pose", joint_pose_ptr);
}

template <typename PointCloudType>
PointCloud::Ptr Ros2Production::convertRsPointCloudToPointCloud(const PointCloudType data) {

  
  auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud_msg->header.stamp = data->header.stamp;
  cloud_msg->height = data->height;
  cloud_msg->width = data->width;
  cloud_msg->is_bigendian = data->is_bigendian;
  cloud_msg->point_step = data->point_step;
  cloud_msg->row_step = data->row_step;
  cloud_msg->is_dense = data->is_dense;
  cloud_msg->fields.resize(data->fields.size());
  for (size_t i = 0; i < data->fields.size(); i++) {
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

  // sensor_msgs::msg::PointCloud2->pcl::PointCloud<pcl::PointXYZ>
  PointCloud::Ptr point_cloud_ptr = std::make_shared<PointCloud>();
  point_cloud_ptr->timestamp = data->header.stamp.sec * 1e9 + data->header.stamp.nanosec;
  pcl::fromROSMsg(*cloud_msg, *point_cloud_ptr->data);
  return point_cloud_ptr;
}

void Ros2Production::start() {
  pipeline_ptr_->start();
//  rclcpp::spin(node_ptr_);
}

void Ros2Production::publishPose2d(const Msg::Ptr &msg_ptr) {
  auto pose_array_msg = std::make_unique<geometry_msgs::msg::PoseArray>();
  pose_array_msg->header.frame_id = frame_id_;
  pose_array_msg->header.stamp = rclcpp::Time(msg_ptr->timestamp);
  // 添加所有姿态点
  for (const auto &point : msg_ptr->internal_result_ptr->image_all_pose_points_map[rally::CameraEnum::left_ac_camera]) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = point.x;
    pose.position.y = point.y;
    pose.position.z = point.score;
    pose_array_msg->poses.push_back(pose);
  }
  int lose_nums = 133 - msg_ptr->internal_result_ptr->image_all_pose_points_map[rally::CameraEnum::left_ac_camera].size();
  // 预测点不足133的模型占位符用0代替
  for (size_t i = 0; i < lose_nums; ++i) {
  geometry_msgs::msg::Pose pose;
  pose_array_msg->poses.push_back(pose);
  }
  for (const auto &point : msg_ptr->internal_result_ptr->image_all_pose_points_map[rally::CameraEnum::right_ac_camera]) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = point.x;
    pose.position.y = point.y;
    pose.position.z = point.score;
    pose_array_msg->poses.push_back(pose);
  }
  // 预测点不足133的模型占位符用0代替
  for (size_t i = 0; i < lose_nums; ++i) {
  geometry_msgs::msg::Pose pose;
  pose_array_msg->poses.push_back(pose);
  }

  // 添加2d滤波后关键点
  for (const auto&point : msg_ptr->internal_result_ptr->image_arm_key_points_map[rally::CameraEnum::left_ac_camera]) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = point.x;
    pose.position.y = point.y;
    pose_array_msg->poses.push_back(pose);
  }
  for (const auto&point : msg_ptr->internal_result_ptr->image_arm_key_points_map[rally::CameraEnum::right_ac_camera]) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = point.x;
    pose.position.y = point.y;
    pose_array_msg->poses.push_back(pose);
  }

  // 添加mark点是否检测到标志符
  geometry_msgs::msg::Pose pose_left;
  if (msg_ptr->internal_result_ptr->qr_code_is_detected_map[rally::CameraEnum::left_ac_camera] == 1)
  {
    pose_left.position.x = 1.0f;
  }
  else
  {
    pose_left.position.x = 0.0f;
  }
  pose_array_msg->poses.push_back(pose_left);

  geometry_msgs::msg::Pose pose_right;
  if (msg_ptr->internal_result_ptr->qr_code_is_detected_map[rally::CameraEnum::right_ac_camera] == 1)
  {
    pose_right.position.x = 1.0f;
  }
  else
  {
    pose_right.position.x = 0.0f;
  }
  pose_array_msg->poses.push_back(pose_right);

  geometry_msgs::msg::Pose pose_left_time;
  pose_left_time.position.x = msg_ptr->input_msg_ptr->image_map[rally::CameraEnum::left_ac_camera]->timestamp;
  pose_array_msg->poses.push_back(pose_left_time);

  geometry_msgs::msg::Pose pose_right_time;
  pose_right_time.position.x = msg_ptr->input_msg_ptr->image_map[rally::CameraEnum::right_ac_camera]->timestamp;
  pose_array_msg->poses.push_back(pose_right_time);

  // 发布消息
  pose_2d_pub_->publish(std::move(pose_array_msg));
  AINFO << name() << ": publish 2d pose.";
}



void Ros2Production::publishPose(const Msg::Ptr &msg_ptr) {
  auto pose_array_msg = std::make_unique<geometry_msgs::msg::PoseArray>();
  pose_array_msg->header.frame_id = frame_id_;
  pose_array_msg->header.stamp = rclcpp::Time(msg_ptr->timestamp);
  // 添加所有姿态点
  for (const auto &point : msg_ptr->output_msg_ptr->arm_key_points) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = point.x;
    pose.position.y = point.y;
    pose.position.z = point.z;
    // 设置姿态的orientation（四元数）为默认值
    pose.orientation.w = 1.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose_array_msg->poses.push_back(pose);
  }
  // 将末端执行器位姿信息编码到消息中
  auto& right_hand_pose = msg_ptr->output_msg_ptr->end_pose.first;
  auto& left_hand_pose = msg_ptr->output_msg_ptr->end_pose.second;
  // 右手位置和旋转
  geometry_msgs::msg::Pose right_effector_pose;
  right_effector_pose.position.x = right_hand_pose.first.x();
  right_effector_pose.position.y = right_hand_pose.first.y();
  right_effector_pose.position.z = right_hand_pose.first.z();
  Eigen::Quaternionf right_hand_oriention = right_hand_pose.second * right_rot;
  // 直接使用四元数
  /*right_effector_pose.orientation.x = right_hand_pose.second.x();*/
  /*right_effector_pose.orientation.y = right_hand_pose.second.y();*/
  /*right_effector_pose.orientation.z = right_hand_pose.second.z();*/
  /*right_effector_pose.orientation.w = right_hand_pose.second.w();*/
  right_effector_pose.orientation.x = right_hand_oriention.x();
  right_effector_pose.orientation.y = right_hand_oriention.y();
  right_effector_pose.orientation.z = right_hand_oriention.z();
  right_effector_pose.orientation.w = right_hand_oriention.w();
  // 左手位置和旋转
  geometry_msgs::msg::Pose left_effector_pose;
  left_effector_pose.position.x = left_hand_pose.first.x();
  left_effector_pose.position.y = left_hand_pose.first.y();
  left_effector_pose.position.z = left_hand_pose.first.z();
  Eigen::Quaternionf left_hand_oriention = left_hand_pose.second * left_rot;
  // 直接使用四元数
  left_effector_pose.orientation.x = left_hand_oriention.x();
  left_effector_pose.orientation.y = left_hand_oriention.y();
  left_effector_pose.orientation.z = left_hand_oriention.z();
  left_effector_pose.orientation.w = left_hand_oriention.w();
  // 添加末端执行器位姿到PoseArray消息
  pose_array_msg->poses.push_back(right_effector_pose);
  pose_array_msg->poses.push_back(left_effector_pose);
  // 发布消息
  pose_3d_pub_->publish(std::move(pose_array_msg));
  AINFO << name() << ": publish pose.";
}

void Ros2Production::publishHandEuler(const Msg::Ptr &msg_ptr) {
  sensor_msgs::msg::JointState handeuler_msg;
  handeuler_msg.header.frame_id = frame_id_;
  handeuler_msg.header.stamp = rclcpp::Time(msg_ptr->timestamp);
  for (const auto& v : msg_ptr->output_msg_ptr->finger_angles) {
    handeuler_msg.position.push_back(v);
  }
  hand_euler_pub_->publish(handeuler_msg);
  AINFO << name() << ": publish hand euler.";
}

void Ros2Production::publishFingerDistance(const Msg::Ptr &msg_ptr) {
  auto left_msg = std::make_unique<sensor_msgs::msg::Temperature>();
  auto right_msg = std::make_unique<sensor_msgs::msg::Temperature>();
  left_msg->header.frame_id = frame_id_;
  left_msg->header.stamp = rclcpp::Time(msg_ptr->timestamp);
  left_msg->temperature = msg_ptr->output_msg_ptr->left_finger_distance;
  right_msg->header.frame_id = frame_id_;
  right_msg->header.stamp = rclcpp::Time(msg_ptr->timestamp);
  right_msg->temperature = msg_ptr->output_msg_ptr->right_finger_distance;
  left_finger_distance_pub_->publish(std::move(left_msg));
  right_finger_distance_pub_->publish(std::move(right_msg));
  AINFO << name() << ": publish finger distance.";
}

void Ros2Production::publishCheckHci(const Msg::Ptr &msg_ptr) {
  // 发送进度信息
  float cur_progress = msg_ptr->internal_result_ptr->check_progress;
  if (cur_progress > 100.f) {
    cur_progress = 100.f;
  }
  std_msgs::msg::String progress_msg;
  progress_msg.data = "ac_check:"+std::to_string(cur_progress);
  m_ProgressPublisher_->publish(progress_msg);
  RINFO << "===========================当前检查进度: " << cur_progress << "% =========================";

  if ((!pub_check_end_) && (msg_ptr->internal_result_ptr->check_finish)) {
    pub_check_end_ = true;
    std_msgs::msg::String hci_msg;
    hci_msg.data = "check_finish";
    m_HciPublisher_->publish(hci_msg);
    RINFO << name() << ": publish check finish.";
    return;
  }

  if (!pub_check_start_) {
    pub_check_start_ = true;
    std_msgs::msg::String hci_msg;
    hci_msg.data = "check_start";
    m_HciPublisher_->publish(hci_msg);
    RINFO << name() << ": publish check start.";
    return;
  }
}

void Ros2Production::publishCalibHci(const Msg::Ptr &msg_ptr) {
  // 发送进度信息
  float cur_progress = msg_ptr->internal_result_ptr->calib_progress;
  if (cur_progress > 100.f) {
    cur_progress = 100.f;
  }
  std_msgs::msg::String progress_msg;
  progress_msg.data = "bone_calib:"+std::to_string(cur_progress);
  m_ProgressPublisher_->publish(progress_msg);
  RINFO << "===========================当前标定进度: " << cur_progress << "% =========================";

  if ((!pub_calib_end_) && (msg_ptr->internal_result_ptr->calib_finish_map.at(rally::CameraEnum::left_ac_camera) &&
    msg_ptr->internal_result_ptr->calib_finish_map.at(rally::CameraEnum::right_ac_camera))) {
    pub_calib_end_ = true;
    std_msgs::msg::String hci_msg;
    hci_msg.data = "calib_finish";
    m_HciPublisher_->publish(hci_msg);
    RINFO << name() << ": publish bone calib finish.";
    return;
  }

  if (!pub_calib_start_) {
    pub_calib_start_ = true;
    std_msgs::msg::String hci_msg;
    hci_msg.data = "bone_calib_start";
    m_HciPublisher_->publish(hci_msg);
    RINFO << name() << ": publish bone calib start.";
    return;
  }

  if (!msg_ptr->internal_result_ptr->calib_info.empty()) {
    msg_ptr->internal_result_ptr->calib_info = ""; // 防止重复发送
    std_msgs::msg::String hci_msg;
    hci_msg.data = "bone_calib_warning";
    m_HciPublisher_->publish(hci_msg);
    RWARN << name() << ": publish bone calib warning.";
    return;
  }
}

}
}
