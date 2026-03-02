//
// Created by sti on 2025/6/12.
//

#include "motion_capture/strategy/optical/two_stage/bone_calibration/calibration.h"

namespace robosense {
namespace motion_capture {

void BoneCalibration::init() {
  AINFO << name() << ": start init...";
  auto cfg_node = rally::ConfigureManager::getInstance().getCfgNode();
  std::string collector = cfg_node["operator"].as<std::string>();
  calibration_file_save_path_map_[rally::CameraEnum::left_ac_camera] = std::string(PROJECT_PATH) + "/config/bone/calib_bone_front_left_" + collector + ".yaml";
  calibration_file_save_path_map_[rally::CameraEnum::right_ac_camera] = std::string(PROJECT_PATH) + "/config/bone/calib_bone_front_right_" + collector + ".yaml";
  calib_cnt_map_[rally::CameraEnum::left_ac_camera] = 0;
  calib_cnt_map_[rally::CameraEnum::right_ac_camera] = 0;
  valid_calib_cnt_map_[rally::CameraEnum::left_ac_camera] = 0;
  valid_calib_cnt_map_[rally::CameraEnum::right_ac_camera] = 0;
  calib_bone_sum_len_map_[rally::CameraEnum::left_ac_camera] = std::array<float, 14>{0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
  calib_bone_sum_len_map_[rally::CameraEnum::right_ac_camera] = std::array<float, 14>{0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
  calib_bone_default_ = std::array<float, 14>{0.56, 0.26, 0.26, 0.56, 0.26, 0.26, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  AINFO << name() << ": finish init.";
}

void BoneCalibration::process(const Msg::Ptr& msg_ptr) {
  AINFO << name() << ": start process...";

  float final_percent = (static_cast<float>(valid_calib_cnt_map_[rally::CameraEnum::left_ac_camera]) + 
  static_cast<float>(valid_calib_cnt_map_[rally::CameraEnum::right_ac_camera])) / calib_max_cnt_ * 50.f;
  msg_ptr->internal_result_ptr->calib_progress = final_percent;

  if (msg_ptr->internal_result_ptr->camera_arm_key_points_map[rally::CameraEnum::left_ac_camera].empty() ||
      msg_ptr->internal_result_ptr->camera_arm_key_points_map[rally::CameraEnum::right_ac_camera].empty()) {
    if (msg_ptr->internal_result_ptr->camera_arm_key_points_map[rally::CameraEnum::left_ac_camera].empty()) {
      RWARN << name() << ": left arm key points are empty, skip calibration.";
    }
    if (msg_ptr->internal_result_ptr->camera_arm_key_points_map[rally::CameraEnum::right_ac_camera].empty()) {
      RWARN << name() << ": right arm key points are empty, skip calibration.";
    }
    return;
  }
  calibSingleAC(msg_ptr, rally::CameraEnum::left_ac_camera);
  calibSingleAC(msg_ptr, rally::CameraEnum::right_ac_camera);
  AINFO << name() << ": finish process.";
}

void BoneCalibration::calibSingleAC(const Msg::Ptr& msg_ptr, rally::CameraEnum camera_enum) {
  calib_cnt_map_[camera_enum]++;
  float percent = static_cast<float>(valid_calib_cnt_map_[camera_enum]) / calib_max_cnt_ * 100.f;
  // 更新进度
  if (valid_calib_cnt_map_[camera_enum] == calib_max_cnt_) {
    std::array<float, 14> calib_bone_len_average;
    for (int i = 0; i < 14; ++i) {
      calib_bone_len_average[i] = calib_bone_sum_len_map_[camera_enum][i] / valid_calib_cnt_map_[camera_enum];
    }
    // 打印结果
    AINFO << name() << ": " << rally::kCameraEnum2NameMap.at(camera_enum) << " calibration completed. average bone lengths: "
          << calib_bone_len_average[0] << " "
          << calib_bone_len_average[1] << " "
          << calib_bone_len_average[2] << " "
          << calib_bone_len_average[3] << " "
          << calib_bone_len_average[4] << " "
          << calib_bone_len_average[5] << " "
          << calib_bone_len_average[6] << " "
          << calib_bone_len_average[7];
    // 保存到文件
    YAML::Node node;
    // 将 std::array 存储为 YAML 数组
    for (size_t i = 0; i < 14; ++i) {
      node["bone_len"][i] = calib_bone_len_average[i];
    }
    std::ofstream fout(calibration_file_save_path_map_[camera_enum]);
    fout << node;
    fout.close();
    valid_calib_cnt_map_[camera_enum]++;
    return;
  } else if (valid_calib_cnt_map_[camera_enum] > calib_max_cnt_) {
    msg_ptr->internal_result_ptr->calib_finish_map[camera_enum] = true;
    AINFO << name() << ": " << rally::kCameraEnum2NameMap.at(camera_enum) <<" 人体骨骼标定数据已保存到: " << calibration_file_save_path_map_[camera_enum];
    return;
  } else if (calib_cnt_map_[camera_enum] < 200) {
    AINFO << name() << 
          ": " << rally::kCameraEnum2NameMap.at(camera_enum) << " 准备开始标定，将身体保持以下姿势, 双手呈45度打开... " << static_cast<int>(percent) << "%\n" <<
          "     O     \n"
          "   / | \\   \n"
          "  /  |  \\  \n"
          " /   |   \\ \n"
          "    / \\    \n"
          "    | |   \n"
          "    | |   \n";    
          return;
  } else {
    AINFO << name() << 
          ": " << rally::kCameraEnum2NameMap.at(camera_enum) << " 将身体保持以下姿势, 双手呈45度打开... " << static_cast<int>(percent) << "%\n" <<
          "     O     \n"
          "   / | \\   \n"
          "  /  |  \\  \n"
          " /   |   \\ \n"
          "    / \\    \n"
          "    | |   \n"
          "    | |   \n";
    const auto& camera_arm_key_points = msg_ptr->internal_result_ptr->camera_arm_key_points_map[camera_enum];

    bool valid = true;
    std::array<float, 14> cur_bone;
    for (size_t k = 0; k < opt_line_set_.size(); ++k) {
      int i = opt_line_set_[k].first;
      int j = opt_line_set_[k].second;
      const auto& pi = camera_arm_key_points[i];
      const auto& pj = camera_arm_key_points[j];
      float len = pi.distance(pj);
      float default_len = calib_bone_default_[k];
      if ((len > default_len + 0.56) || (len < default_len - 0.56)) {
        AWARN << name() << ": " << rally::kCameraEnum2NameMap.at(camera_enum) << " 检测到关节序号：" << i <<" 和关节序号 "<< j <<" 异常关节长度 "<< len <<" 请保持静止";
        valid = false;
        break;
      }
      cur_bone[k] = len;
    }
    if (valid) {
      valid_calib_cnt_map_[camera_enum]++;
      for (size_t k = 0; k < 14; ++k) {
        calib_bone_sum_len_map_[camera_enum][k] += cur_bone[k];
      }
    } else {
      cur_abnormal_time_ += 1;
    }

    if ((cur_abnormal_time_ % info_hz_) == 60) {
      cur_abnormal_time_ += 1;
      msg_ptr->internal_result_ptr->calib_info = "检测到异常关节长度";
    }
  }
}

}
}
