//
// Created by sti on 2025/7/17.
//

#include "motion_capture/strategy/optical/two_stage/check_calibration/check_calibration.h"
#include "motion_capture/utils/sensor_manager/sensor_manager.h"

namespace robosense {
namespace motion_capture {

void CheckCalibration::init(const YAML::Node& cfg_node) {
  AINFO << name() << ": start init...";

  result_save_path_ = std::string(PROJECT_PATH) + "/config/sensor/check_result.yaml";

  time_recorder_ptr_ = std::make_shared<TimeRecorder>(name());
  AINFO << name() << ": finish init.";
}

void CheckCalibration::process(const Msg::Ptr &msg_ptr) {
  time_recorder_ptr_->tic();
  AINFO << name() << ": start process...";
  
  if (msg_ptr->internal_result_ptr->world_arm_key_points_map[rally::CameraEnum::left_ac_camera].empty() ||
      msg_ptr->internal_result_ptr->world_arm_key_points_map[rally::CameraEnum::right_ac_camera].empty()) {
    if (msg_ptr->internal_result_ptr->camera_arm_key_points_map[rally::CameraEnum::left_ac_camera].empty()) {
      RWARN << name() << ": left arm key points are empty, skip calibration.";
    }
    if (msg_ptr->internal_result_ptr->camera_arm_key_points_map[rally::CameraEnum::right_ac_camera].empty()) {
      RWARN << name() << ": right arm key points are empty, skip calibration.";
    }
    return;
  }

  // 更新进度信息
  msg_ptr->internal_result_ptr->check_progress = (float)(cur_frame_+1) / after_frame_end_num_ * 100.f;;

  if (cur_frame_ >= after_frame_end_num_) {
    cur_frame_ = 0;
    cur_status_id_ += 1;
  }

  if (cur_status_id_ >= pose_status_.size()) {
    // 检查标定结果
    CheckResult();
    msg_ptr->internal_result_ptr->check_finish = true;
    AINFO << name() << ": " << " 双AC标定结果已检查完成";
    return;  
  }

  AINFO << name() << ": " << " ==========================================================================";
  AINFO << name() << ": " << " =========================================================================="; 
  AINFO << name() << ": " << " ==========================================================================";
  AINFO << name() << ": " << " ==========================================================================";
  AINFO << name() << ": " << " ==========================================================================";
  AINFO << name() << ": " << " =====================请保持姿势: " << pose_status_[cur_status_id_] << "==================";
  AINFO << name() << ": " << " ==========================================================================";
  AINFO << name() << ": " << " ==========================================================================";
  AINFO << name() << ": " << " ==========================================================================";  
  AINFO << name() << ": " << " ==========================================================================";
  AINFO << name() << ": " << " =========================================================================="; 

  get_pose_frame_points(pose_status_[cur_status_id_], msg_ptr);
  cur_frame_ += 1;

  time_recorder_ptr_->toc();
}

void CheckCalibration::CheckResult() {
  AINFO << name() << ": " << " 开始检查检测结果";

  // 检查双AC之间的标定误差
  size_t frame_num = 0;
  float dis_ac_points = 0.0;
  float dis_x = 0.0;
  float dis_y = 0.0;
  float dis_z = 0.0;

  std::map<std::string, std::vector<float>> status_points;
  for (const auto& pose_status: pose_status_) {
    const auto& cur_status_frame_error = frame_error_dict_[pose_status];

    for (int i=0; i<cur_status_frame_error.size(); ++i) {
      float cur_ac_error = cur_status_frame_error[i][0];
      float cur_x_error = cur_status_frame_error[i][1];
      float cur_y_error = cur_status_frame_error[i][2];
      float cur_z_error = cur_status_frame_error[i][3];

      dis_ac_points += cur_ac_error;
      dis_x += cur_x_error;
      dis_y += cur_y_error;
      dis_z += cur_z_error;
      frame_num += 1;
    }
  }

  float ac_error = dis_ac_points / frame_num;
  float dis_average_x = dis_x / frame_num;
  float dis_average_y = dis_y / frame_num;
  float dis_average_z = dis_z / frame_num;

  bool check_status = true;
  if ((ac_error > ac_error_thre_) || (dis_average_x > x_error_thre_) ||
    (dis_average_y > y_error_thre_) || (dis_average_z > z_error_thre_)) {
    check_status = false;
  }

  // 计算世界中心
  float world_center_x = 0.f;
  float world_center_y = 0.f;
  float world_center_z = 0.f;
  for (const auto& center_point : world_center_points_) {
    world_center_x += center_point[0];
    world_center_y += center_point[1];
    world_center_z += center_point[2];
  }
  world_center_x /= world_center_points_.size();
  world_center_y /= world_center_points_.size();
  world_center_z /= world_center_points_.size();

  // 保存关键指标到文件中
  YAML::Node node;
  node["ac_error"] = ac_error;
  node["dis_average_x"] = dis_average_x;
  node["dis_average_y"] = dis_average_y;
  node["dis_average_z"] = dis_average_z;
  node["check_status"] = check_status;
  node["world_center_x"] = world_center_x;
  node["world_center_y"] = world_center_y;
  node["world_center_z"] = world_center_z;

  RWARN << name() << ": " << " ac_error: " << ac_error;
  RWARN << name() << ": " << " dis_x: " << dis_average_x << " dis_y: " << dis_average_y << " dis_z: " << dis_average_z;
  RWARN << name() << ": " << " check_status: " << check_status;
  RWARN << name() << ": " << " center_x: " << world_center_x << " center_y: " << world_center_y << " center_z: " << world_center_z;

  std::ofstream fout(result_save_path_);
  fout << node;
  fout.close();
}

void CheckCalibration::get_pose_frame_points(const std::string& pose_status,
                                              const Msg::Ptr &msg_ptr) {
    if (cur_frame_ == 0) {
      // 发送语音提示信息
      AINFO << name() << ": " << " 当前检查状态为: " << pose_status;
    }

    if ((cur_frame_ >= before_frame_end_num_) && (cur_frame_ < key_frame_end_num_)) {
      // 保存关键帧信息
      const auto& world_right_key_points_vec = msg_ptr->internal_result_ptr->world_arm_key_points_map[rally::CameraEnum::right_ac_camera];
      const auto& world_left_key_points_vec = msg_ptr->internal_result_ptr->world_arm_key_points_map[rally::CameraEnum::left_ac_camera];
      // 每个AC的关键点检测数量是固定的
      size_t size_right = world_right_key_points_vec.size();
      size_t size_left = world_left_key_points_vec.size();
      if ((size_right != 9) || (size_left != 9)) {
        RWARN << name() << ": " << "exist unvalid detect result!";
        return;
      }
      
      // 计算当前帧中点
      float world_center_x = (world_right_key_points_vec[1].x + world_right_key_points_vec[4].x
                              + world_left_key_points_vec[1].x + world_left_key_points_vec[4].x) / 4;
      float world_center_y = (world_right_key_points_vec[1].y + world_right_key_points_vec[4].y
                              + world_left_key_points_vec[1].y + world_left_key_points_vec[4].y) / 4;
      float world_center_z = (world_right_key_points_vec[1].z + world_right_key_points_vec[4].z
                              + world_left_key_points_vec[1].z + world_left_key_points_vec[4].z) / 4;
      std::vector<float> cur_world_center{world_center_x, world_center_y, world_center_z};
      world_center_points_.emplace_back(cur_world_center);

      // 计算当前帧误差
      // 双AC标定误差
      float ac_error = 0.0f;
      for (int i: compute_pose_id_) {
        float cur_ac_error = world_right_key_points_vec[i].distance(world_left_key_points_vec[i]);
        if (cur_ac_error > 1.0) {
          RWARN << name() << ": " << " 存在左右AC误差较大的点";
          cur_ac_error = 1.0;
        }
        RWARN << name() << ": " << i << " error: " << cur_ac_error;
        ac_error += cur_ac_error;
      }
      ac_error /= compute_pose_id_.size();

      // 世界坐标系误差
      std::vector<float> world_error_x;
      std::vector<float> world_error_y;
      std::vector<float> world_error_z;
      for (auto [id1, id2]: pose_y_connect_id_map_) {
        // 理论上与y轴平行
        const auto& p1 = world_right_key_points_vec[id1];
        const auto& p2 = world_right_key_points_vec[id2];
        float cur_dis_x = abs(p1.x - p2.x);
        float cur_dis_z = abs(p1.z - p2.z);
        world_error_x.emplace_back(cur_dis_x);
        world_error_z.emplace_back(cur_dis_z);
      }

      for (auto [id1, id2]: pose_z_connect_id_map_) {
        // 理论上与z轴平行
        const auto& p1 = world_right_key_points_vec[id1];
        const auto& p2 = world_right_key_points_vec[id2];
        float cur_dis_x = abs(p1.x - p2.x);
        float cur_dis_y = abs(p1.y - p2.y);
        world_error_x.emplace_back(cur_dis_x);
        world_error_y.emplace_back(cur_dis_y);
      }

      // 计算平均误差
      float world_average_error_x = std::accumulate(world_error_x.begin(), world_error_x.end(), 0.0f) / world_error_x.size();
      float world_average_error_y = std::accumulate(world_error_y.begin(), world_error_y.end(), 0.0f) / world_error_y.size();
      float world_average_error_z = std::accumulate(world_error_z.begin(), world_error_z.end(), 0.0f) / world_error_z.size();
      
      std::vector<float> cur_frame_error;
      cur_frame_error.emplace_back(ac_error);
      cur_frame_error.emplace_back(world_average_error_x);
      cur_frame_error.emplace_back(world_average_error_y);
      cur_frame_error.emplace_back(world_average_error_z);
      frame_error_dict_[pose_status].emplace_back(cur_frame_error);
    }
}

} // motion_capture
} // end robosense
