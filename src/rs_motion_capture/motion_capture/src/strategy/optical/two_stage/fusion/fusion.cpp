//
// Created by sti on 2025/6/11.
//

#include "motion_capture/strategy/optical/two_stage/fusion/fusion.h"
#include "motion_capture/utils/sensor_manager/sensor_manager.h"

namespace robosense {
namespace motion_capture {

void PoseFusionOptimization::init(const YAML::Node& cfg_node) {
  AINFO << name() << ": start init...";
  // 加载骨骼标定文件
  const auto& node = rally::ConfigureManager::getInstance().getCfgNode();
  std::string collector = node["operator"].as<std::string>();
  std::string left_bone_calib_file = std::string(PROJECT_PATH) + "/config/bone/calib_bone_front_left_" + collector + ".yaml";
  std::string right_bone_calib_file = std::string(PROJECT_PATH) + "/config/bone/calib_bone_front_right_" + collector + ".yaml";
  YAML::Node left_bone_calib_node, right_bone_calib_node;
  if (!rally::loadFile(left_bone_calib_file, left_bone_calib_node) ||
      !rally::loadFile(right_bone_calib_file, right_bone_calib_node)) {
    RTHROW("Failed to load bone calibration files: " + left_bone_calib_file + " or " + right_bone_calib_file +
      ". Make sure the files exist and are correctly formatted.");
  }
  rally::loadFile(left_bone_calib_file, left_bone_calib_node);
  rally::loadFile(right_bone_calib_file, right_bone_calib_node);
  bone_len_map_.at(rally::CameraEnum::left_ac_camera) =
          left_bone_calib_node["bone_len"].as<std::vector<float>>();
  bone_len_map_.at(rally::CameraEnum::right_ac_camera) =
          right_bone_calib_node["bone_len"].as<std::vector<float>>();
  bone_fusion_len_.resize(bone_len_map_.begin()->second.size(), 0.0f);
  for (const auto& pair : bone_len_map_) {
    const auto& bone_len = pair.second;
    for (size_t i = 0; i < bone_len.size(); ++i) {
      bone_fusion_len_[i] += bone_len[i];
    }
  }
  for (size_t i = 0; i < bone_fusion_len_.size(); ++i) {
    bone_fusion_len_[i] /= bone_len_map_.size();
  }

  const auto& global_cfg_node = rally::ConfigureManager::getInstance().getCfgNode();
  glove_type_ = global_cfg_node["glove_type"].as<std::string>();

  // get transformation from global_cfg_node
  float ws_center_z = global_cfg_node["ws_center_z"].as<float>();
  float world_center_x = global_cfg_node["world_center_x"].as<float>();
  float world_center_y = global_cfg_node["world_center_y"].as<float>();
  float world_center_z = global_cfg_node["world_center_z"].as<float>();
  world_x_bias_ = world_center_x;
  world_y_bias_ = world_center_y;
  world_z_bias_ = world_center_z- ws_center_z;
  AINFO << name() << ": world_x_bias_: " << world_x_bias_;
  AINFO << name() << ": world_y_bias_: " << world_y_bias_;
  AINFO << name() << ": world_z_bias_: " << world_z_bias_;
  time_recorder_ptr_ = std::make_shared<TimeRecorder>(name());
  AINFO << name() << ": finish init.";
}

void PoseFusionOptimization::process(const Msg::Ptr &msg_ptr) {
  time_recorder_ptr_->tic();
  AINFO << name() << ": start process...";
  if (msg_ptr->internal_result_ptr->world_arm_key_points_map[rally::CameraEnum::left_ac_camera].empty() ||
      msg_ptr->internal_result_ptr->world_arm_key_points_map[rally::CameraEnum::right_ac_camera].empty()) {
    if (msg_ptr->internal_result_ptr->world_arm_key_points_map[rally::CameraEnum::left_ac_camera].empty()) {
      AWARN << name() << " left arm key points are empty, skip fusion optimization.";
    }
    if (msg_ptr->internal_result_ptr->world_arm_key_points_map[rally::CameraEnum::right_ac_camera].empty()) {
      AWARN << name() << " right arm key points are empty, skip fusion optimization.";
    }
    pose3d_fusion_history_.clear();
    msg_ptr->valid_flag = false;
    return;
  }

  // 1. 获取融合的初始的3D位姿
  std::vector<Point3f> pose3d_init = getFusionPoseInit(msg_ptr);
  // 2. 执行优化
  std::vector<Point3f> pose3d_optimized = use_opt_ ? ceresSolverOptimization(msg_ptr, pose3d_init) : pose3d_init;
  pose3d_fusion_history_ = pose3d_optimized;
  // 3. 计算末端
  auto end_effector_poses = calculateEndEffectorPose(msg_ptr, pose3d_optimized);
  msg_ptr->internal_result_ptr->fusion_optimized_world_arm_key_points = pose3d_optimized;
  msg_ptr->internal_result_ptr->fusion_optimized_world_end_pose = end_effector_poses;

  // 4. 转到以人体中心为原点的坐标系(pose、end_effector_pose)
  // std::vector<Point3f> pose3d_optimized_origin = pose3d_optimized;
  // Point3f origin = pose3d_optimized_origin[0];
  // for (auto &point : pose3d_optimized_origin) {
  //   point.x -= origin.x;
  //   point.y -= origin.y;
  //   point.z -= origin.z;
  // }

  // 4.转到以自检过程中两肩中点为原点，并将z固定在一个高度
  std::vector<Point3f> pose3d_optimized_origin = pose3d_optimized;
  for (auto &point : pose3d_optimized_origin) {
    point.x -= world_x_bias_;
    point.y -= world_y_bias_;
    point.z -= world_z_bias_;
  }

  // AINFO << "X: " << pose3d_optimized_origin[1].x << " Y: " << pose3d_optimized_origin[1].y << " Z: " << pose3d_optimized_origin[1].z;

  auto end_effector_poses_origin = calculateEndEffectorPose(msg_ptr, pose3d_optimized_origin);
  msg_ptr->output_msg_ptr->arm_key_points = pose3d_optimized_origin;
  msg_ptr->output_msg_ptr->end_pose = end_effector_poses_origin;

  AINFO << name() << ": finish process.";
  time_recorder_ptr_->toc();
}

std::vector<Point3f> PoseFusionOptimization::getFusionPoseInit(const Msg::Ptr &msg_ptr) {
  const auto& world_arm_key_points_map = msg_ptr->internal_result_ptr->world_arm_key_points_map;
  int map_size = world_arm_key_points_map.size();
  std::vector<Point3f> pose3d_init;
  // 假设每个 vector 的长度都相同，取第一个的长度作为基准
  size_t N = world_arm_key_points_map.begin()->second.size();
  pose3d_init.resize(N, Point3f(0.0f, 0.0f, 0.0f));

  for (size_t i = 0; i < N; ++i) {
    std::vector<double> error_value;
    std::vector<const std::vector<Point3f>*> vec_refs;

    bool is_valid = false;

    for (const auto& pair : world_arm_key_points_map) {
      const auto& vec = pair.second;
      vec_refs.push_back(&vec);
      if (!vec[i].valid) {
        break;
      }
      is_valid = true;
      double error_each_pair = 0.0;
      for (size_t j = 0; j < node2bone_[i].size(); ++j) {
        const auto& p1 = vec[node2bone_[i][j].first];
        const auto& p2 = vec[node2bone_[i][j].second];
        float dx = p1.x - p2.x;
        float dy = p1.y - p2.y;
        float dz = p1.z - p2.z;
        float actual_len = std::sqrt(dx * dx + dy * dy + dz * dz);
        float theoretical_len = bone_fusion_len_[node2line_[i][j]];
        float diff = std::abs(actual_len - theoretical_len);
        error_each_pair += diff;
      }
      error_value.push_back(error_each_pair);
    }

    // 计算权重
    std::vector<double> weights(error_value.size(), 0.0);
    double sum_inverse = 0.0;
    for (double e : error_value) {
      if (e > 1e-6) sum_inverse += 1.0 / e;
    }
    for (size_t j = 0; j < error_value.size(); ++j) {
      if (error_value[j] > 1e-6)
        weights[j] = (1.0 / error_value[j]) / sum_inverse;
    }

    // 加权融合
    for (size_t j = 0; j < error_value.size(); ++j) {
      const auto& vec = *vec_refs[j];
      pose3d_init[i].x += vec[i].x * weights[j];
      pose3d_init[i].y += vec[i].y * weights[j];
      pose3d_init[i].z += vec[i].z * weights[j];
    }
    pose3d_init[i].valid = is_valid;

  }
  return pose3d_init;
}

std::vector<Point3f> PoseFusionOptimization::ceresSolverOptimization(const Msg::Ptr &msg_ptr,
                                                                     const std::vector<Point3f> &pose3d_init) {
  size_t N = pose3d_init.size();
  std::vector<std::array<double, 3>> pose3d_param(N);
  for (size_t i = 0; i < N; ++i) {
    pose3d_param[i][0] = pose3d_init[i].x;
    pose3d_param[i][1] = pose3d_init[i].y;
    pose3d_param[i][2] = pose3d_init[i].z;
  }

  ceres::Problem problem;
  for (size_t i = 0; i < N; ++i) {
    problem.AddParameterBlock(pose3d_param[i].data(), 3);
  }
  std::vector<std::pair<ceres::ResidualBlockId, std::string>> residual_info;
  for (const auto& i : opt_idxs_) {
    if (!pose3d_init[i].valid) {
      continue;
    }
    ceres::CostFunction* cost =
            new ceres::AutoDiffCostFunction<SmoothnessResidual, 3, 3>(
                    new SmoothnessResidual(pose3d_init[i], obs3d_beta_));
    auto loss = new ceres::HuberLoss(1.0);
    auto id = problem.AddResidualBlock(cost, loss, pose3d_param[i].data());
    residual_info.emplace_back(id, "obs3d");
  }


  if (!pose3d_fusion_history_.empty()) {
    for (const auto& i : opt_idxs_) {
      if (!pose3d_init[i].valid) {
        continue;
      }
      ceres::CostFunction* cost =
              new ceres::AutoDiffCostFunction<SmoothnessResidual, 3, 3>(
                      new SmoothnessResidual(pose3d_fusion_history_[i], smooth_beta_));
      auto loss = new ceres::HuberLoss(1.0);
      auto id = problem.AddResidualBlock(cost, loss, pose3d_param[i].data());
      residual_info.emplace_back(id, "smooth_fusion");
    }
  }


  for (const auto& camera_enum : camera_list_) {
    const auto& pose2d = msg_ptr->internal_result_ptr->image_arm_key_points_map[camera_enum];
    cv::Mat camera_k = SensorManager::getInstance().getIntrinsic(camera_enum);
    Eigen::Matrix4f camera2world = SensorManager::getInstance().getCamera2World(camera_enum);
    Eigen::Matrix3f camera2world_R = camera2world.block<3, 3>(0, 0);
    Eigen::Vector3f camera2world_T = camera2world.block<3, 1>(0, 3);
    for (const auto& i : opt_idxs_) {
      if (!pose3d_init[i].valid) {
        continue;
      }
      if (!std::isnan(pose2d[i].x) && !std::isnan(pose2d[i].y)) {
        auto loss = new ceres::HuberLoss(1.0);
        ceres::CostFunction* cost =
                new ceres::AutoDiffCostFunction<ProjectionResidual, 2, 3>(
                        new ProjectionResidual(pose2d[i], camera_k, camera2world_R, camera2world_T, gamma_proj_));
        auto id = problem.AddResidualBlock(cost, loss, pose3d_param[i].data());
        residual_info.emplace_back(id, "proj_" + rally::kCameraEnum2NameMap.at(camera_enum));
      }
    }
  }

  for (size_t k = 0; k < opt_line_set_.size(); ++k) {
    int i = opt_line_set_[k].first;
    int j = opt_line_set_[k].second;
    if ((!pose3d_init[i].valid) || (!pose3d_init[j].valid)) {
      continue;
    }
    ceres::CostFunction* cost =
            new ceres::AutoDiffCostFunction<BoneLengthResidual, 1, 3, 3>(
                    new BoneLengthResidual(bone_fusion_len_[k], length_beta_));
    auto id = problem.AddResidualBlock(cost, nullptr,
                                       pose3d_param[i].data(),
                                       pose3d_param[j].data());
    residual_info.emplace_back(id, "bone");
  }

  ceres::Solver::Options options;
  options.max_num_iterations = 20;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  loss_.reset();
  for (const auto& pair : residual_info) {
    ceres::ResidualBlockId id = pair.first;
    const std::string& type = pair.second;
    ceres::Problem::EvaluateOptions options;
    options.apply_loss_function = true;
    options.residual_blocks = {id};
    std::vector<double> residuals;
    double cost;
    problem.Evaluate(options, &cost, &residuals, nullptr, nullptr);
    double block_cost = 0.0;
    for (double r : residuals) {
      block_cost += r * r;
    }
    if (type.find("bone") == 0) {
      loss_.bone_length_cost += block_cost;
    } else if (type.find("proj") == 0) {
      loss_.projection_cost += block_cost;
    } else if (type.find("smooth") == 0) {
      loss_.smoothness_cost += block_cost;
    } else if (type == "obs3d") {
      loss_.obs3d_cost += block_cost;
    }
  }
  loss_.sum();

  std::vector<Point3f> pose3d_optimized(N);
  for (size_t i = 0; i < N; ++i) {
    pose3d_optimized[i].x = static_cast<float>(pose3d_param[i][0]);
    pose3d_optimized[i].y = static_cast<float>(pose3d_param[i][1]);
    pose3d_optimized[i].z = static_cast<float>(pose3d_param[i][2]);
    pose3d_optimized[i].valid = pose3d_init[i].valid;
  }
  return pose3d_optimized;
}

std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, std::pair<Eigen::Vector3f, Eigen::Quaternionf>>
PoseFusionOptimization::calculateEndEffectorPose(const Msg::Ptr &msg_ptr, const std::vector<Point3f> &pose) {
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

Eigen::Quaternionf PoseFusionOptimization::computeHandRotationFromBone(const Eigen::Vector3f &p1,
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
