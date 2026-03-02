//
// Created by sti on 2025/6/6.
//

#include "motion_capture/strategy/optical/two_stage/pose_detection/postprocess.h"
#include "motion_capture/utils/sensor_manager/sensor_manager.h"

namespace robosense {
namespace motion_capture {

void PoseDetectionPostprocess::init(const YAML::Node& cfg_node) {
  AINFO << name() << ": start init...";
  img_attr_.image_width_ = SensorManager::getInstance().getWidth(rally::CameraEnum::left_ac_camera);
  img_attr_.image_height_ = SensorManager::getInstance().getHeight(rally::CameraEnum::left_ac_camera);
  img_attr_.half_width_ = img_attr_.image_width_ / 2;
  img_attr_.half_height_ = img_attr_.image_height_ / 2;
  img_attr_.model_input_width_ = 288;
  img_attr_.model_input_height_ = 384;

  const auto& global_cfg_node = rally::ConfigureManager::getInstance().getCfgNode();
  glove_type_ = global_cfg_node["glove_type"].as<std::string>();

  time_recorder_ptr_ = std::make_shared<TimeRecorder>(name());
  AINFO << name() << ": finish init.";
  YAML::Node kalman_config = cfg_node["kalman"];

  filters_2d_map_[rally::CameraEnum::left_ac_camera] = std::vector<KalmanFilter2D>();
  filters_2d_map_[rally::CameraEnum::right_ac_camera] = std::vector<KalmanFilter2D>();

  filters_1d_map_[rally::CameraEnum::left_ac_camera] =  std::vector<KalmanFilter1D>();
  filters_1d_map_[rally::CameraEnum::right_ac_camera] =  std::vector<KalmanFilter1D>();

  for (size_t i = 0; i < filter_nums_; i++) {
    filters_2d_map_[rally::CameraEnum::left_ac_camera].push_back(KalmanFilter2D(kalman_config["2d_Q"].as<float>(), kalman_config["2d_R"].as<float>()));
    filters_2d_map_[rally::CameraEnum::right_ac_camera].push_back(KalmanFilter2D(kalman_config["2d_Q"].as<float>(), kalman_config["2d_R"].as<float>()));

    filters_1d_map_[rally::CameraEnum::left_ac_camera].push_back(KalmanFilter1D(kalman_config["1d_Q"].as<float>(), kalman_config["1d_R"].as<float>()));
    filters_1d_map_[rally::CameraEnum::right_ac_camera].push_back(KalmanFilter1D(kalman_config["1d_Q"].as<float>(), kalman_config["1d_R"].as<float>()));
  };


}

void PoseDetectionPostprocess::processSingleImpl(const Msg::Ptr& msg_ptr, rally::CameraEnum camera_enum) {
  auto res = getModelOutputKeyPts(msg_ptr, camera_enum);
  msg_ptr->internal_result_ptr->image_all_pose_points_map[camera_enum] = res;
  getAllKeyPts(msg_ptr, res, camera_enum);
}

std::vector<Point2f> PoseDetectionPostprocess::getModelOutputKeyPts(const Msg::Ptr& msg_ptr, rally::CameraEnum camera_enum) {
  const auto& xmin = msg_ptr->internal_result_ptr->pose_model_input_bbox_map[camera_enum].xmin;
  const auto& ymin = msg_ptr->internal_result_ptr->pose_model_input_bbox_map[camera_enum].ymin;
  const auto& xmax = msg_ptr->internal_result_ptr->pose_model_input_bbox_map[camera_enum].xmax;
  const auto& ymax = msg_ptr->internal_result_ptr->pose_model_input_bbox_map[camera_enum].ymax;

  float* simccx = camera_enum == rally::CameraEnum::left_ac_camera
                      ? bindings_ptr_->pd_bindings_host.simcc_x
                      : bindings_ptr_->pd_bindings_host.simcc_x + num_points_ * extend_width_;
  float* simccy = camera_enum == rally::CameraEnum::left_ac_camera
                      ? bindings_ptr_->pd_bindings_host.simcc_y
                      : bindings_ptr_->pd_bindings_host.simcc_y + num_points_ * extend_height_;
  std::vector<Point2f> res;
  res.reserve(num_points_);
  for (int i = 0; i < num_points_; ++i) {
    std::vector<float> simcc_x(simccx + i * extend_width_,
                               simccx + (i + 1) * extend_width_);
    std::vector<float> simcc_y(simccy + i * extend_height_,
                               simccy + (i + 1) * extend_height_);

    // 寻找x方向最大值及其索引
    auto x_max_it = std::max_element(simcc_x.begin(), simcc_x.end());
    float x_loc = std::distance(simcc_x.begin(), x_max_it);
    float max_val_x = *x_max_it;
    // 寻找y方向最大值及其索引
    auto y_max_it = std::max_element(simcc_y.begin(), simcc_y.end());
    float y_loc = std::distance(simcc_y.begin(), y_max_it);
    float max_val_y = *y_max_it;
    // 计算置信度 - 使用x和y的平均值，与Python实现一致
    float score = 0.5f * (max_val_x + max_val_y);

    if (score <= 0.f) {
      res.emplace_back(0.f, 0.f, 0.f, false);
    } else {
      float x = x_loc * simcc_ratio_ / img_attr_.model_input_width_ * (xmax - xmin) + xmin;
      float y = y_loc * simcc_ratio_ / img_attr_.model_input_height_ * (ymax - ymin) + ymin;
//      std::cout << "id: " << i << ", x: " << x << ", y: " << y << ", score: " << score << std::endl;
      bool is_valid = (x >= 0 && x < img_attr_.image_width_ && y >= 0 && y < img_attr_.image_height_) &&
                      (score >= score_threshold_);
      res.emplace_back(x, y, score, is_valid);
    }
  }
  return res;
}

void PoseDetectionPostprocess::getAllKeyPts(const Msg::Ptr& msg_ptr,
                                            const std::vector<Point2f>& res,
                                            rally::CameraEnum camera_enum) {
  bool key_pts_valid = true;
  for (const auto& idx : all_key_pts_idx_) {
    if (!res[idx].is_valid()) {
      key_pts_valid = false;
      AWARN << name() <<" 时间戳 "<<msg_ptr->timestamp<<"  point idx " <<idx<<" invalid, score: "<<res[idx].score<<" x: "<<res[idx].x<<" y: "<<res[idx].y<<" img width: "<<img_attr_.image_width_ << " img height: "<<img_attr_.image_height_;
      break;
    }
  }

//  std::cout << "key_pts_valid: " << key_pts_valid << std::endl;

  if (key_pts_valid) {
    // 1.获取图像坐标系下的关键点
    auto img_arm_key_pts = getImageCoordKeyPts(msg_ptr, res, camera_enum);
    msg_ptr->internal_result_ptr->image_arm_key_points_map[camera_enum] = img_arm_key_pts;
    // 2.获取世界坐标系下的关键点
    auto world_arm_key_pts = getWorldCoordKeyPts(msg_ptr, img_arm_key_pts, camera_enum);
    msg_ptr->internal_result_ptr->world_arm_key_points_map[camera_enum] = world_arm_key_pts;
    // 3.计算末端pose和两指距离
    // todo:是不是没必要？可视化？
    auto end_effector_poses = calculateEndEffectorPose(msg_ptr, world_arm_key_pts);
    msg_ptr->internal_result_ptr->world_end_pose_map[camera_enum] = end_effector_poses;

  } else {
    // 清空历史信息
    last_triangle_pt_valid_map_[camera_enum] = 0;
    ema_key_pts_depth_map_[camera_enum].clear();
  }
}

std::vector<Point2f> PoseDetectionPostprocess::getImageCoordKeyPts(const Msg::Ptr& msg_ptr,
                                                                    const std::vector<Point2f>& res,
                                                                    rally::CameraEnum camera_enum) {
  // 获取所有的图像关键点
  std::vector<Point2f> img_arm_key_pts;
  // 计算人体三角骨架(左肩、右肩、左右髋中点）
  std::vector<cv::Point2f> cur_triangle_point;
  cv::Point2f mid_point(0.f, 0.f);
  for (const auto& idx : hip_key_pts_idx_) {
    mid_point.x += res[idx].x;
    mid_point.y += res[idx].y;
  }
  mid_point.x /= hip_key_pts_idx_.size();
  mid_point.y /= hip_key_pts_idx_.size();
  cur_triangle_point.push_back(mid_point);
  for (const auto& idx : shouder_key_pts_idx_) {
    cur_triangle_point.push_back(cv::Point2f(res[idx].x, res[idx].y));
  }

  // 获取人体中点
  if (use_aruco_) {
    if (msg_ptr->internal_result_ptr->qr_code_is_detected_map[camera_enum] == 1) {
      AINFO << name() << ": 相机检测到二维码, 使用二维码中点作为人体中点";
      img_arm_key_pts.push_back(msg_ptr->internal_result_ptr->center_map[camera_enum]);
      // 保存上一帧的二维码中点
      last_aruco_pt_map_[camera_enum] = msg_ptr->internal_result_ptr->center_map[camera_enum];
    } else {
      // 没有检测到二维码，分两种情况：如果上一帧检测到三角，使用上一帧二维码的仿射变换；否则使用当前帧检测的左右髋中点
      if (last_triangle_pt_valid_map_[camera_enum] == 1) {
        AINFO << name() << ": 相机没有检测到二维码, 使用仿射变换将上一帧的 ArUco 中心点映射到当前帧";
        auto last_triangle_point = last_triangle_pt_map_[camera_enum];
        cv::Mat M = cv::getAffineTransform(last_triangle_point, cur_triangle_point);
        auto last_aruco_pt = last_aruco_pt_map_[camera_enum];
        std::vector<cv::Point2f> input_points = {cv::Point2f(last_aruco_pt.x, last_aruco_pt.y)};
        std::vector<cv::Point2f> output_points;
        cv::transform(input_points, output_points, M);
        img_arm_key_pts.emplace_back(output_points[0].x, output_points[0].y);
        last_aruco_pt_map_[camera_enum].x = output_points[0].x;
        last_aruco_pt_map_[camera_enum].y = output_points[0].y;
      } else {
        RERROR << name() << ": 相机没有检测到二维码, 使用当前帧检测的左右髋中点作为人体中点";
        img_arm_key_pts.emplace_back(mid_point.x, mid_point.y);
        last_aruco_pt_map_[camera_enum].x = mid_point.x;
        last_aruco_pt_map_[camera_enum].y = mid_point.y;
      }
    }
    last_triangle_pt_valid_map_[camera_enum] = 1;
    last_triangle_pt_map_[camera_enum] = cur_triangle_point;
  } else {
    // 没有使用 ArUco，直接使用当前帧检测的左右髋中点作为人体中点
    img_arm_key_pts.emplace_back(mid_point.x, mid_point.y);
  }

  // 人体中点始终valid
  img_arm_key_pts.back().valid = true;

  // 左右肩肘腕
  for (const auto& idx : arm_key_pts_idx_) {
    img_arm_key_pts.emplace_back(res[idx]);
  }

  // 左右手
  // todo: 需要判断是否valid?
  Point2f left_hand_pt;
  for (const auto& idx : left_hand_key_pts_idx_) {
    left_hand_pt.x += res[idx].x;
    left_hand_pt.y += res[idx].y;
  }
  left_hand_pt.x /= left_hand_key_pts_idx_.size();
  left_hand_pt.y /= left_hand_key_pts_idx_.size();
  Point2f right_hand_pt;
  for (const auto& idx : right_hand_key_pts_idx_) {
    right_hand_pt.x += res[idx].x;
    right_hand_pt.y += res[idx].y;
  }
  right_hand_pt.x /= right_hand_key_pts_idx_.size();
  right_hand_pt.y /= right_hand_key_pts_idx_.size();
  img_arm_key_pts.emplace_back(left_hand_pt);
  img_arm_key_pts.emplace_back(right_hand_pt);

  // 下半身关键点
  for (const auto& idx : lower_body_key_pts_idx_) {
    img_arm_key_pts.emplace_back(res[idx]);
  }


  // kalman 2d
  auto& filters_2d = filters_2d_map_[camera_enum];
  for (size_t i = 0; i < filter_nums_; i++)
  {
    auto& filter = filters_2d[i];
    Eigen::Vector2f joint(img_arm_key_pts[i].x, img_arm_key_pts[i].y);
    if (!filter.is_initialized) {
      filter.initialize(joint,
                        static_cast<double>(msg_ptr->input_msg_ptr->image_map[camera_enum]->timestamp) * 1e-9f);
      img_arm_key_pts[i].x = joint.x();
      img_arm_key_pts[i].y = joint.y();
    }
    else {
      filter.predict(static_cast<double>(msg_ptr->input_msg_ptr->image_map[camera_enum]->timestamp) * 1e-9f);
      Eigen::Vector2f smoothed_joint = filter.update(joint);
      img_arm_key_pts[i].x = smoothed_joint.x();
      img_arm_key_pts[i].y = smoothed_joint.y();
    }

  }
  return img_arm_key_pts;
}

std::vector<Point3f> PoseDetectionPostprocess::getWorldCoordKeyPts(const Msg::Ptr &msg_ptr,
                                                                   const std::vector<Point2f> &img_arm_key_pts,
                                                                   const rally::CameraEnum &camera_enum) {
  // 获取所有的对应的相机坐标系关键点
  // 将点云投影到图像上，获取每个点的图像坐标和深度信息
  rally::LidarEnum lidar_enum = camera_enum == rally::CameraEnum::left_ac_camera ?
                                rally::LidarEnum::left_ac_lidar : rally::LidarEnum::right_ac_lidar;
  size_t num_pts = msg_ptr->input_msg_ptr->lidar_map.at(lidar_enum)->data->size();
  pcl::PointCloud<pcl::PointXY>::Ptr cloud_image(new pcl::PointCloud<pcl::PointXY>);
  std::shared_ptr<std::vector<float>> cloud_image_depth(new std::vector<float>());
  cloud_image->reserve(num_pts);
  cloud_image_depth->reserve(num_pts);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_camera(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Matrix4f transform = SensorManager::getInstance().getLidar2Camera(lidar_enum);
  pcl::transformPointCloud(*msg_ptr->input_msg_ptr->lidar_map.at(lidar_enum)->data, *cloud_camera, transform);

  cv::Mat intrinsic = SensorManager::getInstance().getResizeIntrinsic(camera_enum,
                                                                      img_attr_.image_height_,
                                                                      img_attr_.image_width_);

  float fx = intrinsic.at<float>(0, 0);
  float fy = intrinsic.at<float>(1, 1);
  float cx = intrinsic.at<float>(0, 2);
  float cy = intrinsic.at<float>(1, 2);
  for (const auto& pt : cloud_camera->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z) || pt.z <= 0.0f || pt.z >= 10.0f) {
      continue;
    }
    float inv_z = 1.0f / pt.z;
    float u = ((fx * pt.x) * inv_z + cx);
    float v = ((fy * pt.y) * inv_z + cy);
    if (u >= 0 && u < img_attr_.image_width_ && v >= 0 && v < img_attr_.image_height_) {
      pcl::PointXY point_xy;
      point_xy.x = u;
      point_xy.y = v;
      cloud_image->points.emplace_back(point_xy);
      cloud_image_depth->emplace_back(pt.z);
    }
  }
  // 添加到中间结果
  msg_ptr->internal_result_ptr->image_projected_pc_map[camera_enum] = cloud_image;
  msg_ptr->internal_result_ptr->image_projected_pc_depth_map[camera_enum] = cloud_image_depth;

  // 查找图像上的关键点对应的深度值
  std::vector<float> key_pts_depth;
  pcl::KdTreeFLANN<pcl::PointXY> kdtree;
  kdtree.setInputCloud(cloud_image);

  for (const auto& pt : img_arm_key_pts) {
    std::vector<int> indices(4);
    std::vector<float> sq_distances(4);
    pcl::PointXY searchPoint;
    searchPoint.x = pt.x;
    searchPoint.y = pt.y;
    int find_k = kdtree.nearestKSearch(searchPoint, 4, indices, sq_distances);
    if (find_k > 0) {
      float sum_z = 0.0f;
      for (size_t i = 0; i < find_k; i++) {
        sum_z += cloud_image_depth->at(indices[i]);
      }
      key_pts_depth.emplace_back(sum_z/find_k);
    }
  }

  auto& filters_1d = filters_1d_map_[camera_enum];

  if (ema_key_pts_depth_map_[camera_enum].empty()) {
    ema_key_pts_depth_map_[camera_enum] = key_pts_depth;
  } else {
    uint8_t smooth_st=0;
    if (use_aruco_ && msg_ptr->internal_result_ptr->qr_code_is_detected_map[camera_enum] == 0) {
      smooth_st = 1;
    }
    // 平滑处理
    for (size_t i = smooth_st; i < key_pts_depth.size(); i++) {
      auto& filter = filters_1d[i];
      filter.predict(static_cast<double>(msg_ptr->input_msg_ptr->image_map[camera_enum]->timestamp) * 1e-9f);
      ema_key_pts_depth_map_[camera_enum][i] = filter.update(key_pts_depth[i]);
    }
  }

  // 转换为相机坐标系坐标
  std::vector<Point3f> camera_arm_key_pts;
  camera_arm_key_pts.reserve(img_arm_key_pts.size());
  cv::Mat uv1(img_arm_key_pts.size(), 3, CV_32F);
  for (size_t i = 0; i < img_arm_key_pts.size(); i++) {
    uv1.at<float>(i, 0) = img_arm_key_pts[i].x;
    uv1.at<float>(i, 1) = img_arm_key_pts[i].y;
    uv1.at<float>(i, 2) = 1.0;
  }

  cv::Mat intrinsic_inv = intrinsic.inv();
  cv::Mat cam_coords = (intrinsic_inv * uv1.t()).t();
  for (size_t i = 0; i < img_arm_key_pts.size(); i++) {
    float x = cam_coords.at<float>(i, 0) * ema_key_pts_depth_map_[camera_enum][i];
    float y = cam_coords.at<float>(i, 1) * ema_key_pts_depth_map_[camera_enum][i];
    float z = ema_key_pts_depth_map_[camera_enum][i];
    camera_arm_key_pts.emplace_back(x, y, z, img_arm_key_pts[i].score, img_arm_key_pts[i].valid);
  }

  // 添加到中间结果
  msg_ptr->internal_result_ptr->camera_arm_key_points_map[camera_enum] = camera_arm_key_pts;

  // 获取所有的对应的世界坐标系关键点
  std::vector<Point3f> world_arm_key_pts;
  for (size_t i = 0; i < camera_arm_key_pts.size(); i++) {
    Eigen::Vector4f camera_pt(camera_arm_key_pts[i].x, camera_arm_key_pts[i].y, camera_arm_key_pts[i].z, 1.0);
    Eigen::Vector4f world_pt = SensorManager::getInstance().getCamera2World(camera_enum) * camera_pt;
    world_arm_key_pts.emplace_back(world_pt.x(), world_pt.y(), world_pt.z(),
                                   camera_arm_key_pts[i].score, camera_arm_key_pts[i].valid);
  }
  return world_arm_key_pts;
}

void PoseDetectionPostprocess::process(const Msg::Ptr &msg_ptr) {
  time_recorder_ptr_->tic();
  AINFO << name() << ": start process...";
  if (msg_ptr->internal_result_ptr->is_person_detected_map[rally::CameraEnum::left_ac_camera] == 0 ||
      msg_ptr->internal_result_ptr->is_person_detected_map[rally::CameraEnum::right_ac_camera] == 0) {
    if (msg_ptr->internal_result_ptr->is_person_detected_map[rally::CameraEnum::left_ac_camera] == 0) {
      RWARN << name() << ": left ac camera not detected, skipping pose detection!";
    }
    if (msg_ptr->internal_result_ptr->is_person_detected_map[rally::CameraEnum::right_ac_camera] == 0) {
      RWARN << name() << ": right ac camera not detected, skipping pose detection!";
    }
    return;
  }

  processSingleImpl(msg_ptr, rally::CameraEnum::left_ac_camera);
  processSingleImpl(msg_ptr, rally::CameraEnum::right_ac_camera);
  AINFO << name() << ": finish process.";
  time_recorder_ptr_->toc();
}

std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, std::pair<Eigen::Vector3f, Eigen::Quaternionf>>
PoseDetectionPostprocess::calculateEndEffectorPose(const Msg::Ptr &msg_ptr, const std::vector<Point3f> &pose) {
  Eigen::Vector3f right_position = Eigen::Vector3f::Zero();
  Eigen::Quaternionf right_rotation = Eigen::Quaternionf::Identity();
  Eigen::Vector3f left_position = Eigen::Vector3f::Zero();
  Eigen::Quaternionf left_rotation = Eigen::Quaternionf::Identity();

  if (pose.size() >= 9) {
    // 对于右手 (点索引：1, 2, 3)
    // 提取点坐标
    Eigen::Vector3f p1(pose[1].x, pose[1].y, pose[1].z);
    Eigen::Vector3f p2(pose[2].x, pose[2].y, pose[2].z);
    Eigen::Vector3f p3(pose[3].x, pose[3].y, pose[3].z);
    // 对于左手 (点索引：4, 5, 6)
    // 提取点坐标
    Eigen::Vector3f p4(pose[4].x, pose[4].y, pose[4].z);
    Eigen::Vector3f p5(pose[5].x, pose[5].y, pose[5].z);
    Eigen::Vector3f p6(pose[6].x, pose[6].y, pose[6].z);
    // 右手位置 (p3)
    right_position = p3;
    // 左手位置 (p6)
    left_position = p6;
    if (msg_ptr->use_glove) {
      if ("manus" == glove_type_) {
        left_rotation = Eigen::Quaternionf(msg_ptr->input_msg_ptr->hand_joints_pose2->data->at(3),
                                           msg_ptr->input_msg_ptr->hand_joints_pose2->data->at(0),
                                           msg_ptr->input_msg_ptr->hand_joints_pose2->data->at(1),
                                           msg_ptr->input_msg_ptr->hand_joints_pose2->data->at(2));
        right_rotation = Eigen::Quaternionf(msg_ptr->input_msg_ptr->hand_joints_pose2->data->at(13),
                                            msg_ptr->input_msg_ptr->hand_joints_pose2->data->at(10),
                                            msg_ptr->input_msg_ptr->hand_joints_pose2->data->at(11),
                                            msg_ptr->input_msg_ptr->hand_joints_pose2->data->at(12));
      } else if ("noiton" == glove_type_) {
        const auto& left_pose = msg_ptr->input_msg_ptr->hand_joints_pose->data->at(0);
        const auto& right_pose = msg_ptr->input_msg_ptr->hand_joints_pose->data->at(20);

        Eigen::Quaternionf right_quat(right_pose.w(),
                                      right_pose.x(),
                                      right_pose.y(),
                                      right_pose.z());
        // 转换为欧拉角（Z-Y-X，即 yaw-pitch-roll）
        Eigen::Vector3f right_euler = right_quat.toRotationMatrix().eulerAngles(2, 1, 0); // yaw, pitch, roll

        // 修改 yaw 和 pitch（绕 Z 和 Y 轴）为反向
        float yaw   = right_euler[0]; // 原本绕Z轴的角度
        float pitch = -right_euler[1]; // 原本绕Y轴的角度
        float roll  = -right_euler[2]; // 保持绕X轴不变

        // 重新构造四元数：注意顺序是 Z-Y-X
        Eigen::AngleAxisf roll_angle(roll,  Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitch_angle(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yaw_angle(yaw,   Eigen::Vector3f::UnitZ());

        right_quat = yaw_angle * pitch_angle * roll_angle;

        // // 初始位置x朝下
        Eigen::Quaternionf q_world_rot(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitY()));
        right_quat = q_world_rot * right_quat;
        Eigen::Matrix3f right_R = right_quat.toRotationMatrix();
        // x轴替换为z轴
        Eigen::Matrix3f R_local;
        R_local.col(0) = Eigen::Vector3f(0, 1, 0);
        R_local.col(1) = Eigen::Vector3f(0, 0, 1);
        R_local.col(2) = Eigen::Vector3f(1, 0, 0);
        right_R = right_R * R_local;
        // 转回四元数
        right_rotation = Eigen::Quaternionf(right_R);
        right_rotation.normalize();
        // 获取左手位置和旋转
        Eigen::Quaternionf left_quat(left_pose.w(),
                                     left_pose.x(),
                                     left_pose.y(),
                                     left_pose.z());
        // 初始位置x朝下
        left_quat = q_world_rot * left_quat;
        Eigen::Matrix3f left_R = (left_quat).toRotationMatrix();
        // x轴替换为z轴
        left_R = left_R * R_local;
        left_rotation = Eigen::Quaternionf(left_R);
        left_rotation.normalize();
      } else {
        RERROR << name() << ": unsupported glove type: " << glove_type_;
      }
    } else {
      right_rotation = computeHandRotationFromBone(p1, p2, p3);
      left_rotation = computeHandRotationFromBone(p4, p5, p6);
    }
  }
  return {{right_position, right_rotation}, {left_position, left_rotation}};
}

Eigen::Quaternionf PoseDetectionPostprocess::computeHandRotationFromBone(const Eigen::Vector3f &p1,
                                                           const Eigen::Vector3f &p2,
                                                           const Eigen::Vector3f &p3) {
  Eigen::Vector3f vec21 = (p2 - p1).normalized();
  Eigen::Vector3f vec32 = (p3 - p2).normalized();
  Eigen::Vector3f normal = vec32.cross(vec21).normalized();

  Eigen::Vector3f Z = vec32;
  Eigen::Vector3f X, Y;

  if (normal.norm() < 1e-4) {
    Eigen::Vector3f rel_y{0.0, 0.0, 1.0};
    X = (rel_y.cross(Z)).normalized();
    Y = Z.cross(X);
  } else {
    X = normal;
    Y = (Z.cross(X)).normalized();
    if (Y.z() < 0) {
      X = -X;
      Y = -Y;
    }
  }

  Eigen::Matrix3f R;
  R.col(0) = X;
  R.col(1) = Y;
  R.col(2) = Z;

  Eigen::Quaternionf q(R);
  return q.normalized();
}

}
}
