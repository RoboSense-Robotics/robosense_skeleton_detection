//
// Created by sti on 2025/6/13.
//

#include "rviz_display/rviz_display.h"
#include <cuda_runtime.h>
#include <filesystem>
#include <opencv2/imgcodecs.hpp>
#include "motion_capture/common/macros.h"

namespace fs = std::filesystem;

namespace robosense {
namespace motion_capture {

void RvizDisplay::init() {
  const auto& rviz_node = rally::ConfigureManager::getInstance().getCfgNode()["ros"]["rviz"];

  // vis pose_3d coord
  change_coord_ = rviz_node["change_coord"].as<bool>();
  const auto& global_cfg_node = rally::ConfigureManager::getInstance().getCfgNode();
  float ws_center_z = global_cfg_node["ws_center_z"].as<float>();
  float world_center_x = global_cfg_node["world_center_x"].as<float>();
  float world_center_y = global_cfg_node["world_center_y"].as<float>();
  float world_center_z = global_cfg_node["world_center_z"].as<float>();
  world_x_bias_ = world_center_x;
  world_y_bias_ = world_center_y;
  world_z_bias_ = world_center_z - ws_center_z;

  controller_.left_vis_enable = rviz_node["left_ac_vis"]["enable"].as<bool>();
  controller_.left_od_enable = rviz_node["left_ac_vis"]["left_ac_od"]["enable"].as<bool>();
  controller_.left_pd_enable = rviz_node["left_ac_vis"]["left_ac_pd"]["enable"].as<bool>();
  controller_.left_lidar_enable = rviz_node["left_ac_vis"]["left_ac_lidar"]["enable"].as<bool>();
  controller_.left_3d_marker_enable = rviz_node["left_ac_vis"]["left_ac_3d"]["enable"].as<bool>();
  controller_.right_vis_enable = rviz_node["right_ac_vis"]["enable"].as<bool>();
  controller_.right_od_enable = rviz_node["right_ac_vis"]["right_ac_od"]["enable"].as<bool>();
  controller_.right_pd_enable = rviz_node["right_ac_vis"]["right_ac_pd"]["enable"].as<bool>();
  controller_.right_lidar_enable = rviz_node["right_ac_vis"]["right_ac_lidar"]["enable"].as<bool>();
  controller_.right_3d_marker_enable = rviz_node["right_ac_vis"]["right_ac_3d"]["enable"].as<bool>();
  controller_.fusion_vis_enable = rviz_node["fusion_vis"]["enable"].as<bool>();
  controller_.fusion_3d_marker_enable = rviz_node["fusion_vis"]["fusion_3d"]["enable"].as<bool>();

  left_od_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::Image>(
          rviz_node["left_ac_vis"]["left_ac_od"]["topic"].as<std::string>(), 10);
  right_od_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::Image>(
          rviz_node["right_ac_vis"]["right_ac_od"]["topic"].as<std::string>(), 10);
  left_pd_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::Image>(
          rviz_node["left_ac_vis"]["left_ac_pd"]["topic"].as<std::string>(), 10);
  right_pd_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::Image>(
          rviz_node["right_ac_vis"]["right_ac_pd"]["topic"].as<std::string>(), 10);
  left_3d_marker_pub_ = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>(
          rviz_node["left_ac_vis"]["left_ac_3d"]["topic"].as<std::string>(), 10);
  right_3d_marker_pub_ = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>(
          rviz_node["right_ac_vis"]["right_ac_3d"]["topic"].as<std::string>(), 10);
  fusion_3d_marker_pub_ = node_ptr_->create_publisher<visualization_msgs::msg::MarkerArray>(
          rviz_node["fusion_vis"]["fusion_3d"]["topic"].as<std::string>(), 10);
  left_lidar_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(
          rviz_node["left_ac_vis"]["left_ac_lidar"]["topic"].as<std::string>(), 10);
  right_lidar_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(
          rviz_node["right_ac_vis"]["right_ac_lidar"]["topic"].as<std::string>(), 10);

  ac_name_color_map_[rally::CameraEnum::left_ac_camera] = createColor(0.f, 1.f, 0.f, 1.f);
  ac_name_color_map_[rally::CameraEnum::right_ac_camera] = createColor(0.f, 0.f, 1.f, 1.f);
  ac_name_color_map_[rally::CameraEnum::fusion] = createColor(1.f, 0.f, 0.f, 1.f);

  uint32_t image_width = SensorManager::getInstance().getWidth(rally::CameraEnum::left_ac_camera);
  uint32_t image_height = SensorManager::getInstance().getHeight(rally::CameraEnum::left_ac_camera);
  ori_img_data_.resize(image_width * image_height * 3, 0);

  // dump config
  save_root_path_ = rviz_node["save_root_path"].as<std::string>();
  left_pd_dump_ = rviz_node["left_ac_vis"]["left_ac_pd"]["dump"].as<bool>();
  left_pd_dir_ = (fs::path(save_root_path_) / rviz_node["left_ac_vis"]["left_ac_pd"]["dump_dir"].as<std::string>() / "left_camera").string();
  if(!fs::exists(left_pd_dir_) && controller_.left_vis_enable && left_pd_dump_){
    fs::create_directories(left_pd_dir_);
  }
  right_pd_dump_ = rviz_node["right_ac_vis"]["right_ac_pd"]["dump"].as<bool>();
  right_pd_dir_ = (fs::path(save_root_path_) / rviz_node["right_ac_vis"]["right_ac_pd"]["dump_dir"].as<std::string>() / "right_camera").string();
  if(!fs::exists(right_pd_dir_) && controller_.right_vis_enable && right_pd_dump_){
    fs::create_directories(right_pd_dir_);
  }

  worker_->bind([this](const Msg::Ptr &msg_ptr) {
      this->display(msg_ptr);
      for (auto &cb : this->display_cb_list_) {
        cb(msg_ptr);
      }
  });
}

void RvizDisplay::display(const Msg::Ptr &msg_ptr) {
  if (controller_.left_vis_enable) {
    if (controller_.left_od_enable) {
      displayODResult(msg_ptr, rally::CameraEnum::left_ac_camera);
    }
    if (controller_.left_pd_enable) {
      displayPDResult(msg_ptr, rally::CameraEnum::left_ac_camera);
    }
    if (controller_.left_lidar_enable) {
      displayLidar(msg_ptr, rally::CameraEnum::left_ac_camera);
    }
    if (controller_.left_3d_marker_enable) {
      display3DMarker(msg_ptr, rally::CameraEnum::left_ac_camera);
    }
  }
  if (controller_.right_vis_enable) {
    if (controller_.right_od_enable) {
      displayODResult(msg_ptr, rally::CameraEnum::right_ac_camera);
    }
    if (controller_.right_pd_enable) {
      displayPDResult(msg_ptr, rally::CameraEnum::right_ac_camera);
    }
    if (controller_.right_lidar_enable) {
      displayLidar(msg_ptr, rally::CameraEnum::right_ac_camera);
    }
    if (controller_.right_3d_marker_enable) {
      display3DMarker(msg_ptr, rally::CameraEnum::right_ac_camera);
    }
  }
  if (controller_.fusion_vis_enable) {
    if (controller_.fusion_3d_marker_enable) {
      display3DMarker(msg_ptr, rally::CameraEnum::fusion);
    }
  }
}

void RvizDisplay::displayODResult(const Msg::Ptr &msg_ptr, rally::CameraEnum camera_enum) {
  const auto& inter_res_ptr = msg_ptr->internal_result_ptr;
  uint32_t image_width = SensorManager::getInstance().getWidth(rally::CameraEnum::left_ac_camera);
  uint32_t image_height = SensorManager::getInstance().getHeight(rally::CameraEnum::left_ac_camera);
  cv::Mat image;
  cv::Mat ori_image = camera_enum == rally::CameraEnum::left_ac_camera ? inter_res_ptr->left_undistort_image : inter_res_ptr->right_undistort_image;
  cv::cvtColor(ori_image,
               image, cv::COLOR_RGB2BGR);

  // draw 2d bbox
  if (inter_res_ptr->is_person_detected_map[camera_enum] == 1) {
    // 画rect
    cv::rectangle(image,
                  cv::Point(static_cast<int>(inter_res_ptr->detected_person_bbox_map[camera_enum].xmin),
                            static_cast<int>(inter_res_ptr->detected_person_bbox_map[camera_enum].ymin)),
                  cv::Point(static_cast<int>(inter_res_ptr->detected_person_bbox_map[camera_enum].xmax),
                            static_cast<int>(inter_res_ptr->detected_person_bbox_map[camera_enum].ymax)),
                  cv::Scalar(0, 255, 0), 5);
  }
  std_msgs::msg::Header header;
  header.stamp = rclcpp::Time(msg_ptr->timestamp);
  header.frame_id = frame_id_;
  sensor_msgs::msg::Image::SharedPtr image_msg =
          cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  if (camera_enum == rally::CameraEnum::left_ac_camera) {
    left_od_pub_->publish(*image_msg);
  } else if (camera_enum == rally::CameraEnum::right_ac_camera) {
    right_od_pub_->publish(*image_msg);
  } else {
    RWARN << "Invalid camera enum for display OD result.";
  }
}

void RvizDisplay::displayPDResult(const Msg::Ptr &msg_ptr, rally::CameraEnum camera_enum) {
  const auto& inter_res_ptr = msg_ptr->internal_result_ptr;
  const auto& pc_z = inter_res_ptr->image_projected_pc_depth_map[camera_enum];
  const auto& pc_ptr = inter_res_ptr->image_projected_pc_map[camera_enum];
  if (!pc_ptr || !pc_z) {
    return;
  }

  uint32_t image_width = SensorManager::getInstance().getWidth(rally::CameraEnum::left_ac_camera);
  uint32_t image_height = SensorManager::getInstance().getHeight(rally::CameraEnum::left_ac_camera);
  cv::Mat image;

  cv::Mat ori_image = camera_enum == rally::CameraEnum::left_ac_camera ? inter_res_ptr->left_undistort_image : inter_res_ptr->right_undistort_image;
  cv::cvtColor(ori_image,
               image, cv::COLOR_RGB2BGR);


  // 0. 绘制二维码
  if (msg_ptr->internal_result_ptr->qr_code_is_detected_map[camera_enum] == 1) {
    const auto& center = msg_ptr->internal_result_ptr->center_map[camera_enum];
    // 画圆
    cv::circle(image,
               cv::Point(static_cast<int>(center.x), static_cast<int>(center.y)),
               20, cv::Scalar(0, 0, 255), 10);
  }

  // 1. 网络2d点连线
  const auto& points = inter_res_ptr->image_all_pose_points_map[camera_enum];
  for (const auto& joint_link : halpe_26_joint_links) {
    size_t idx1 = static_cast<size_t>(joint_link.first);
    size_t idx2 = static_cast<size_t>(joint_link.second);

    // 确保索引在有效范围内且点是有效的
    if (idx1 < points.size() && idx2 < points.size() &&
        points[idx1].valid && points[idx2].valid) {
      cv::line(
              image,
              cv::Point(int(points[idx1].x ), int(points[idx1].y)),
              cv::Point(int(points[idx2].x ), int(points[idx2].y )),
              cv::Scalar(0, 255, 0),
              1,
              cv::LINE_AA);
    }
  }
  // draw points and scores
  for (size_t i = 0; i < points.size(); i++) {
    if (points[i].valid) {
      // 画关键点
      cv::circle(
              image,
              cv::Point(int(points[i].x ), int(points[i].y )),
              1,
              cv::Scalar(0, 0, 255),
              1,
              cv::LINE_AA);
    }
  }

  // 2. 激光点投影到图像上的坐标和深度可视化
  // cv::Mat depth_image = cv::Mat::zeros(image.size(), CV_8UC3);
  // cv::Scalar min_depth_color(255, 0, 0); // 蓝色 - 近
  // cv::Scalar max_depth_color(0, 0, 255); // 红色 - 远
  // float min_depth = std::numeric_limits<float>::max();
  // float max_depth = 0.0f;
  // for (size_t i = 0; i < pc_z->size(); i++) {
  //   if (pc_z->at(i) > 0 && pc_z->at(i) < min_depth) {
  //     min_depth = pc_z->at(i);
  //   }
  //   if (pc_z->at(i) > max_depth) {
  //     max_depth = pc_z->at(i);
  //   }
  // }
  // // 增强深度对比度
  // float depth_range = max_depth - min_depth;
  // float contrast_min = min_depth;
  // float contrast_max = max_depth;

  // // 如果深度范围过大，可以调整对比度范围
  // /*if (depth_range > 5.0f) {*/
  // /*  contrast_min = min_depth + depth_range * 0.1f;*/
  // /*  contrast_max = max_depth - depth_range * 0.1f;*/
  // /*}*/
  // // min max fixed
  // contrast_min = 0.5;
  // contrast_max = 4.5;

  // 绘制深度图
  // for (size_t i = 0; i < pc_ptr->size(); i++) {
  //   int x = pc_ptr->at(i).x;
  //   int y = pc_ptr->at(i).y;
  //   if (x >= 0 && x < depth_image.cols && y >= 0 && y < depth_image.rows) {
  //     // 归一化深度值，增强对比度
  //     float normalized_depth = (pc_z->at(i) - contrast_min) / (contrast_max - contrast_min);
  //     normalized_depth = std::max(0.0f, std::min(1.0f, normalized_depth));
  //     // 使用更鲜明的颜色映射
  //     cv::Scalar color;
  //     if (normalized_depth < 0.25f) {
  //       // 蓝色到青色
  //       float t = normalized_depth * 4.0f;
  //       color = cv::Scalar(255, 255 * t, 0);
  //     } else if (normalized_depth < 0.5f) {
  //       // 青色到绿色
  //       float t = (normalized_depth - 0.25f) * 4.0f;
  //       color = cv::Scalar(255 * (1.0f - t), 255, 0);
  //     } else if (normalized_depth < 0.75f) {
  //       // 绿色到黄色
  //       float t = (normalized_depth - 0.5f) * 4.0f;
  //       color = cv::Scalar(0, 255, 255 * t);
  //     } else {
  //       // 黄色到红色
  //       float t = (normalized_depth - 0.75f) * 4.0f;
  //       color = cv::Scalar(0, 255 * (1.0f - t), 255);
  //     }
  //     // 绘制更大的点
  //     cv::circle(depth_image, cv::Point(x, y), 2, color, -1);
  //   }
  // }

  // // 将深度图混合到原始图像
  // cv::Mat depth_overlay;
  // cv::addWeighted(image, 0.6, depth_image, 0.4, 0, depth_overlay);

  cv::Mat depth_overlay = image;

  // 绘制提取的手臂关键点
  const auto& arm_keypoints = inter_res_ptr->image_arm_key_points_map[camera_enum];
  for (size_t idx = 0; idx < display_points_idx_.size(); ++idx) {
    int i = display_points_idx_[idx];
    if (!arm_keypoints[i].valid) {
      continue;
    }
    cv::circle(depth_overlay, cv::Point(static_cast<int>(arm_keypoints[i].x),
                                        static_cast<int>(arm_keypoints[i].y)) , 5, cv::Scalar(255, 0, 255), -1);
   // std::string label = std::to_string(idx)+ " : "+ cv::format("%.2f",arm_keypoints[i].score);
    std::string label = std::to_string(idx);
    cv::putText(depth_overlay, label,
                cv::Point(static_cast<int>(arm_keypoints[i].x) + 5, static_cast<int>(arm_keypoints[i].y) - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255), 2);
  }

  for (const auto& line : line_set_) {
    int i = line.first;
    int j = line.second;
    if (!arm_keypoints[i].valid || !arm_keypoints[j].valid) {
      continue;
    }
    cv::line(depth_overlay, cv::Point(static_cast<int>(arm_keypoints[i].x),
                                        static_cast<int>(arm_keypoints[i].y)), cv::Point(static_cast<int>(arm_keypoints[j].x),
                                        static_cast<int>(arm_keypoints[j].y)), cv::Scalar(255, 0, 255), 2);
  }

  // 添加图例
  cv::putText(depth_overlay, "Depth: Blue(near) -> Red(far)",
              cv::Point(20, 30),
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
  cv::putText(depth_overlay, "Pose Keypoints: Green",
              cv::Point(20, 60),
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
  cv::putText(depth_overlay, "Arm Keypoints: Pink",
              cv::Point(20, 90),
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255), 2);

  // 显示深度范围
  // std::string depth_range_str = cv::format("Depth Range: %.2f - %.2f m", min_depth, max_depth);
  // cv::putText(depth_overlay, depth_range_str,
  //             cv::Point(20, 120),
  //             cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

  // 显示时间戳
  cv::putText(depth_overlay, "TimeStamp: " + std::to_string(msg_ptr->timestamp),
              cv::Point(20, 150),
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

  std_msgs::msg::Header header;
  header.stamp = rclcpp::Time(msg_ptr->timestamp);
  header.frame_id = frame_id_;
  sensor_msgs::msg::Image::SharedPtr image_msg =
          cv_bridge::CvImage(header, "bgr8", depth_overlay).toImageMsg();
  if (camera_enum == rally::CameraEnum::left_ac_camera) {
    left_pd_pub_->publish(*image_msg);
    if(left_pd_dump_){
      cv::imwrite(left_pd_dir_+"/"+std::to_string(msg_ptr->timestamp)+".jpg", depth_overlay);
    }
  } else if (camera_enum == rally::CameraEnum::right_ac_camera) {
    right_pd_pub_->publish(*image_msg);
    if(right_pd_dump_){
      cv::imwrite(right_pd_dir_+"/"+std::to_string(msg_ptr->timestamp)+".jpg", depth_overlay);
    }
  } else {
    RWARN << "Invalid camera enum for display PD result.";
  }
}

void RvizDisplay::display3DMarker(const Msg::Ptr &msg_ptr, rally::CameraEnum camera_enum) {
  const auto& inter_res_ptr = msg_ptr->internal_result_ptr;
  if (camera_enum == rally::CameraEnum::left_ac_camera ||
      camera_enum == rally::CameraEnum::right_ac_camera) {
    if (inter_res_ptr->world_arm_key_points_map[camera_enum].empty()) {
      return;
    }
  } else {
    if (inter_res_ptr->fusion_optimized_world_arm_key_points.empty()) {
      return;
    }
  }

  visualization_msgs::msg::MarkerArray marker_array;
  // 创建点标记
  visualization_msgs::msg::Marker points_marker;
  points_marker.header.frame_id = frame_id_;
  points_marker.header.stamp = rclcpp::Time(msg_ptr->timestamp);
  points_marker.ns = "pose_points";
  points_marker.action = visualization_msgs::msg::Marker::ADD;
  points_marker.pose.orientation.w = 1.0;
  points_marker.id = 0;
  points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  points_marker.scale.x = 0.02; // 点的大小 (5cm)
  points_marker.scale.y = 0.02;
  points_marker.scale.z = 0.02;
  points_marker.color = ac_name_color_map_.at(camera_enum);

  // 创建线标记 - 在关键点0-1, 1-2, 2-3之间构建连线
  visualization_msgs::msg::Marker lines_marker;
  lines_marker.header.frame_id = frame_id_;
  lines_marker.header.stamp = rclcpp::Time(msg_ptr->timestamp);
  lines_marker.ns = "pose_lines";
  lines_marker.action = visualization_msgs::msg::Marker::ADD;
  lines_marker.pose.orientation.w = 1.0;
  lines_marker.id = 1;
  lines_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  lines_marker.scale.x = 0.01; // 线宽 (2cm)
  lines_marker.color = ac_name_color_map_.at(camera_enum);

  // 添加文本标记 - 显示关键点ID
  std::vector<visualization_msgs::msg::Marker> text_markers;
  // 是否需要在3D空间进行缩放，根据数据分布情况决定
  const float scale_factor = 1.0; // 缩放因子
  // 添加所有点
  const auto& pose_3d = (camera_enum == rally::CameraEnum::fusion ?
                        inter_res_ptr->fusion_optimized_world_arm_key_points :
                        inter_res_ptr->world_arm_key_points_map[camera_enum]);
  for (size_t idx = 0; idx < display_points_idx_.size(); ++idx) {
    int i = display_points_idx_[idx];
    if (!pose_3d[i].valid) {
      continue;
    }
    geometry_msgs::msg::Point p;
    if (change_coord_) {
      p.x = (pose_3d[i].x - world_x_bias_)  * scale_factor;
      p.y = (pose_3d[i].y - world_y_bias_) * scale_factor;
      p.z = (pose_3d[i].z - world_z_bias_) * scale_factor;
    } else {
      p.x = pose_3d[i].x * scale_factor;
      p.y = pose_3d[i].y * scale_factor;
      p.z = pose_3d[i].z * scale_factor;
    }
    points_marker.points.push_back(p);
    // 创建文本标记
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = frame_id_;
    text_marker.header.stamp = rclcpp::Time(msg_ptr->timestamp);
    text_marker.ns = "pose_text";
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.id = static_cast<int>(i + 10); // ID从10开始避免冲突
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.pose.position = p;
    text_marker.pose.position.z += 0.1; // 文本向上偏移一点
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.1; // 文本大小
    text_marker.color = ac_name_color_map_.at(camera_enum);
    text_marker.text = std::to_string(idx);
    text_markers.push_back(text_marker);
  }

  // 添加连线
  for (const auto& line : line_set_) {
    int i = line.first;
    int j = line.second;
    geometry_msgs::msg::Point pi, pj;
    if (!pose_3d[i].valid || !pose_3d[j].valid) {
      continue;
    }
    if (change_coord_) {
      pi.x = (pose_3d[i].x - world_x_bias_) * scale_factor;
      pi.y = (pose_3d[i].y - world_y_bias_) * scale_factor;
      pi.z = (pose_3d[i].z - world_z_bias_) * scale_factor;
      pj.x = (pose_3d[j].x - world_x_bias_) * scale_factor;
      pj.y = (pose_3d[j].y - world_y_bias_) * scale_factor;
      pj.z = (pose_3d[j].z - world_z_bias_) * scale_factor;
    } else {
      pi.x = pose_3d[i].x * scale_factor;
      pi.y = pose_3d[i].y * scale_factor;
      pi.z = pose_3d[i].z * scale_factor;
      pj.x = pose_3d[j].x * scale_factor;
      pj.y = pose_3d[j].y * scale_factor;
      pj.z = pose_3d[j].z * scale_factor;
    }
    lines_marker.points.push_back(pi);
    lines_marker.points.push_back(pj);
  }

  // ================================================= 添加原点坐标系
  visualization_msgs::msg::Marker origin_marker;
  origin_marker.header.frame_id = frame_id_;
  origin_marker.header.stamp = rclcpp::Time(msg_ptr->timestamp);
  origin_marker.ns = "origin_axes";
  origin_marker.action = visualization_msgs::msg::Marker::ADD;
  origin_marker.id = 2;
  origin_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  origin_marker.scale.x = 0.02; // 线宽
  origin_marker.color.a = 1.0;  // 不透明度
  // 原点
  geometry_msgs::msg::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;
  // X轴 - 红色
  geometry_msgs::msg::Point x_axis;
  x_axis.x = 0.3; // 30cm长
  x_axis.y = 0.0;
  x_axis.z = 0.0;
  origin_marker.points.push_back(origin);
  origin_marker.points.push_back(x_axis);
  origin_marker.colors.push_back(createColor(1.0, 0.0, 0.0, 1.0)); // 红色
  origin_marker.colors.push_back(createColor(1.0, 0.0, 0.0, 1.0));
  // Y轴 - 绿色
  geometry_msgs::msg::Point y_axis;
  y_axis.x = 0.0;
  y_axis.y = 0.3;
  y_axis.z = 0.0;
  origin_marker.points.push_back(origin);
  origin_marker.points.push_back(y_axis);
  origin_marker.colors.push_back(createColor(0.0, 1.0, 0.0, 1.0)); // 绿色
  origin_marker.colors.push_back(createColor(0.0, 1.0, 0.0, 1.0));
  // Z轴 - 蓝色
  geometry_msgs::msg::Point z_axis;
  z_axis.x = 0.0;
  z_axis.y = 0.0;
  z_axis.z = 0.3;
  origin_marker.points.push_back(origin);
  origin_marker.points.push_back(z_axis);
  origin_marker.colors.push_back(createColor(0.0, 0.0, 1.0, 1.0)); // 蓝色
  origin_marker.colors.push_back(createColor(0.0, 0.0, 1.0, 1.0));

  // ================================================= 添加右手末端点坐标系
  // 获取右手位置和旋转
  const auto& right_pose = (camera_enum == rally::CameraEnum::fusion ?
                           inter_res_ptr->fusion_optimized_world_end_pose.first :
                           inter_res_ptr->world_end_pose_map[camera_enum].first);
  Eigen::Vector3f right_position = right_pose.first;
  Eigen::Quaternionf right_quat = right_pose.second;
  // 创建右手坐标系标记
  visualization_msgs::msg::Marker right_axes_marker;
  right_axes_marker.header.frame_id = frame_id_;
  right_axes_marker.header.stamp = rclcpp::Time(msg_ptr->timestamp);
  right_axes_marker.ns = "right_hand_axes";
  right_axes_marker.action = visualization_msgs::msg::Marker::ADD;
  right_axes_marker.id = 3;
  right_axes_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  right_axes_marker.scale.x = 0.02; // 线宽
  right_axes_marker.color.a = 1.0;  // 不透明度
  // 右手原点
  geometry_msgs::msg::Point right_origin;
  if (change_coord_) {
    right_origin.x = right_position.x() - world_x_bias_;
    right_origin.y = right_position.y() - world_y_bias_;
    right_origin.z = right_position.z() - world_z_bias_;
  } else {
    right_origin.x = right_position.x();
    right_origin.y = right_position.y();
    right_origin.z = right_position.z();
  }

  // 将四元数转换为旋转矩阵
  Eigen::Matrix3f right_R = right_quat.toRotationMatrix();
  // 坐标轴长度
  const double axis_length = 0.2; // 20cm
  // X轴 - 红色
  Eigen::Vector3f right_x_dir = right_R.col(0) * axis_length;
  geometry_msgs::msg::Point right_x_axis;
  right_x_axis.x = right_origin.x + right_x_dir.x();
  right_x_axis.y = right_origin.y + right_x_dir.y();
  right_x_axis.z = right_origin.z + right_x_dir.z();
  right_axes_marker.points.push_back(right_origin);
  right_axes_marker.points.push_back(right_x_axis);
  right_axes_marker.colors.push_back(createColor(1.0, 0.0, 0.0, 1.0)); // 红色
  right_axes_marker.colors.push_back(createColor(1.0, 0.0, 0.0, 1.0));
  // Y轴 - 绿色
  Eigen::Vector3f right_y_dir = right_R.col(1) * axis_length;
  geometry_msgs::msg::Point right_y_axis;
  right_y_axis.x = right_origin.x + right_y_dir.x();
  right_y_axis.y = right_origin.y + right_y_dir.y();
  right_y_axis.z = right_origin.z + right_y_dir.z();
  right_axes_marker.points.push_back(right_origin);
  right_axes_marker.points.push_back(right_y_axis);
  right_axes_marker.colors.push_back(createColor(0.0, 1.0, 0.0, 1.0)); // 绿色
  right_axes_marker.colors.push_back(createColor(0.0, 1.0, 0.0, 1.0));
  // Z轴 - 蓝色
  Eigen::Vector3f right_z_dir = right_R.col(2) * axis_length;
  geometry_msgs::msg::Point right_z_axis;
  right_z_axis.x = right_origin.x + right_z_dir.x();
  right_z_axis.y = right_origin.y + right_z_dir.y();
  right_z_axis.z = right_origin.z + right_z_dir.z();
  right_axes_marker.points.push_back(right_origin);
  right_axes_marker.points.push_back(right_z_axis);
  right_axes_marker.colors.push_back(createColor(0.0, 0.0, 1.0, 1.0)); // 蓝色
  right_axes_marker.colors.push_back(createColor(0.0, 0.0, 1.0, 1.0));

  // ================================================= 添加左手末端点坐标系
  // 获取左手位置和旋转
  auto left_pose = (camera_enum == rally::CameraEnum::fusion ?
                    inter_res_ptr->fusion_optimized_world_end_pose.second :
                    inter_res_ptr->world_end_pose_map[camera_enum].second);
  Eigen::Vector3f left_position = left_pose.first;
  Eigen::Quaternionf left_quat = left_pose.second;
  // 创建左手坐标系标记
  visualization_msgs::msg::Marker left_axes_marker;
  left_axes_marker.header.frame_id = frame_id_;
  left_axes_marker.header.stamp = rclcpp::Time(msg_ptr->timestamp);
  left_axes_marker.ns = "left_hand_axes";
  left_axes_marker.action = visualization_msgs::msg::Marker::ADD;
  left_axes_marker.id = 4;
  left_axes_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  left_axes_marker.scale.x = 0.02; // 线宽
  left_axes_marker.color.a = 1.0;  // 不透明度
  // 左手原点
  geometry_msgs::msg::Point left_origin;
  if (change_coord_) {
    left_origin.x = left_position.x() - world_x_bias_;
    left_origin.y = left_position.y() - world_y_bias_;
    left_origin.z = left_position.z() - world_z_bias_;
  } else {
    left_origin.x = left_position.x();
    left_origin.y = left_position.y();
    left_origin.z = left_position.z();
  }

  // 将四元数转换为旋转矩阵
  Eigen::Matrix3f left_R = left_quat.toRotationMatrix();
  // X轴 - 红色
  Eigen::Vector3f left_x_dir = left_R.col(0) * axis_length;
  geometry_msgs::msg::Point left_x_axis;
  left_x_axis.x = left_origin.x + left_x_dir.x();
  left_x_axis.y = left_origin.y + left_x_dir.y();
  left_x_axis.z = left_origin.z + left_x_dir.z();
  left_axes_marker.points.push_back(left_origin);
  left_axes_marker.points.push_back(left_x_axis);
  left_axes_marker.colors.push_back(createColor(1.0, 0.0, 0.0, 1.0)); // 红色
  left_axes_marker.colors.push_back(createColor(1.0, 0.0, 0.0, 1.0));
  // Y轴 - 绿色
  Eigen::Vector3f left_y_dir = left_R.col(1) * axis_length;
  geometry_msgs::msg::Point left_y_axis;
  left_y_axis.x = left_origin.x + left_y_dir.x();
  left_y_axis.y = left_origin.y + left_y_dir.y();
  left_y_axis.z = left_origin.z + left_y_dir.z();
  left_axes_marker.points.push_back(left_origin);
  left_axes_marker.points.push_back(left_y_axis);
  left_axes_marker.colors.push_back(createColor(0.0, 1.0, 0.0, 1.0)); // 绿色
  left_axes_marker.colors.push_back(createColor(0.0, 1.0, 0.0, 1.0));
  // Z轴 - 蓝色
  Eigen::Vector3f left_z_dir = left_R.col(2) * axis_length;
  geometry_msgs::msg::Point left_z_axis;
  left_z_axis.x = left_origin.x + left_z_dir.x();
  left_z_axis.y = left_origin.y + left_z_dir.y();
  left_z_axis.z = left_origin.z + left_z_dir.z();
  left_axes_marker.points.push_back(left_origin);
  left_axes_marker.points.push_back(left_z_axis);
  left_axes_marker.colors.push_back(createColor(0.0, 0.0, 1.0, 1.0)); // 蓝色
  left_axes_marker.colors.push_back(createColor(0.0, 0.0, 1.0, 1.0));

  // 添加所有标记到数组
  marker_array.markers.push_back(points_marker);
  marker_array.markers.push_back(lines_marker);
  marker_array.markers.push_back(origin_marker);
  marker_array.markers.push_back(right_axes_marker);
  marker_array.markers.push_back(left_axes_marker);
  for (const auto &marker : text_markers) {
    marker_array.markers.push_back(marker);
  }

  // 发布标记数组
  if (camera_enum == rally::CameraEnum::fusion) {
    fusion_3d_marker_pub_->publish(marker_array);
  } else if (camera_enum == rally::CameraEnum::right_ac_camera) {
    right_3d_marker_pub_->publish(marker_array);
  } else if (camera_enum == rally::CameraEnum::left_ac_camera) {
    left_3d_marker_pub_->publish(marker_array);
  } else {
    RWARN << "Invalid camera enum for display 3D marker result.";
  }
}

void RvizDisplay::displayLidar(const Msg::Ptr &msg_ptr, rally::CameraEnum lidar_enum) {
  const auto& inter_res_ptr = msg_ptr->internal_result_ptr;

  const auto& pc_z = inter_res_ptr->image_projected_pc_depth_map[lidar_enum];
  const auto& pc_ptr = inter_res_ptr->image_projected_pc_map[lidar_enum];

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width  = pc_z->size();
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);
  RINFO<<   "pc size: "<<pc_ptr->size()<<", pc_z size: "<<pc_z->size();
  for (size_t i = 0; i < cloud.points.size(); ++i) {
      cloud.points[i].x = pc_ptr->at(i).x;
      cloud.points[i].y = pc_ptr->at(i).y;
      cloud.points[i].z = pc_z->at(i);
  }
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = frame_id_;
  cloud_msg.header.stamp = rclcpp::Time(msg_ptr->timestamp);
  if (lidar_enum == rally::CameraEnum::left_ac_camera) {
    left_lidar_pub_->publish(cloud_msg);
  } else if (lidar_enum == rally::CameraEnum::right_ac_camera) {
    right_lidar_pub_->publish(cloud_msg);
  } else {
    RWARN << "Invalid lidar enum for display lidar data.";
  }
  // const auto& pcl_lidar_ptr = msg_ptr->input_msg_ptr->lidar_map[lidar_enum]->data;
  // // project to world coordinate
  // Eigen::Matrix4f lidar2world_transform = SensorManager::getInstance().getLidar2World(lidar_enum);
  // pcl::transformPointCloud(*pcl_lidar_ptr, *pcl_lidar_ptr, lidar2world_transform);
  // sensor_msgs::msg::PointCloud2 cloud_msg;
  // pcl::toROSMsg(*pcl_lidar_ptr, cloud_msg);
  // cloud_msg.header.frame_id = frame_id_;
  // cloud_msg.header.stamp = rclcpp::Time(msg_ptr->timestamp);
  // if (lidar_enum == rally::LidarEnum::left_ac_lidar) {
  //   left_lidar_pub_->publish(cloud_msg);
  // } else if (lidar_enum == rally::LidarEnum::right_ac_lidar) {
  //   right_lidar_pub_->publish(cloud_msg);
  // } else {
  //   RWARN << "Invalid lidar enum for display lidar data.";
  // }
}

}
}
