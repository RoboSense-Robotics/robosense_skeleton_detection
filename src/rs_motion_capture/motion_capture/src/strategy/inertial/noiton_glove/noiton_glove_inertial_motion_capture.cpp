//
// Created by sti on 2025/6/12.
//

#include "motion_capture/strategy/inertial/noiton_glove/noiton_glove_inertial_motion_capture.h"

namespace robosense {
namespace motion_capture {

void NoitonGloveInertialMotionCapture::init(const YAML::Node& cfg_node) {

}

void NoitonGloveInertialMotionCapture::process(const Msg::Ptr& msg_ptr) {
  float left_catch_deg = 0.0f;
  float right_catch_deg = 0.0f;
  std::vector<size_t> left_catch_indices = {1, 5, 9 ,13, 17};
  for (size_t pose_idx : left_catch_indices){
    const auto& pose = msg_ptr->input_msg_ptr->hand_joints_pose->data->at(pose_idx);
    // 提取四元数
    Eigen::Quaternionf q(
            pose.w(),
            pose.x(),
            pose.y(),
            pose.z()
    );
    // 变换为欧拉角 (yaw-pitch-roll)，顺序为 ZYX
    Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(2, 1, 0); // yaw, pitch, roll
    float yaw_rad = euler[0];
    float yaw_deg = yaw_rad / M_PI * 180.0;
    float pitch_rad = euler[1];
    float pitch_deg = pitch_rad / M_PI * 180.0;
    float roll_rad = euler[2];
    float roll_deg = roll_rad / M_PI * 180.0;
    if (pose_idx == 1){
      msg_ptr->output_msg_ptr->finger_angles.push_back(yaw_deg);
      msg_ptr->output_msg_ptr->finger_angles.push_back(pitch_deg);
      msg_ptr->output_msg_ptr->finger_angles.push_back(roll_deg);
    }
    else {
      if (yaw_deg < 90) {
        yaw_deg = 180 + yaw_deg;
      }
      msg_ptr->output_msg_ptr->finger_angles.push_back(yaw_deg);
      if (pose_idx == 5 || pose_idx ==9) {
        left_catch_deg += (yaw_deg - 90);
      }
    }
  }
  left_catch_deg /= 2;
  if (left_catch_deg > 110){
    left_catch_deg = 110;
  }
  if (left_catch_deg < 0){
    left_catch_deg = 0;
  }
  float left_finger_distance_mean = left_catch_deg / 110;

  std::vector<size_t> right_catch_indices = {21, 25, 29, 33 ,37};
  for (size_t pose_idx : right_catch_indices){
    const auto& pose = msg_ptr->input_msg_ptr->hand_joints_pose->data->at(pose_idx);
    // 提取四元数
    Eigen::Quaternionf q(
            pose.w(),
            pose.x(),
            pose.y(),
            pose.z()
    );
    // 变换为欧拉角 (yaw-pitch-roll)，顺序为 ZYX
    Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(2, 1, 0); // yaw, pitch, roll
    float yaw_rad = euler[0];
    float yaw_deg = yaw_rad / M_PI * 180.0;
    float pitch_rad = euler[1];
    float pitch_deg = pitch_rad / M_PI * 180.0;
    float roll_rad = euler[2];
    float roll_deg = roll_rad / M_PI * 180.0;
    if (pose_idx == 21){
      msg_ptr->output_msg_ptr->finger_angles.push_back(yaw_deg);
      msg_ptr->output_msg_ptr->finger_angles.push_back(pitch_deg);
      msg_ptr->output_msg_ptr->finger_angles.push_back(roll_deg);
    }
    else {
      if (yaw_deg < 90) {
        yaw_deg = 90 - yaw_deg;
      }
      else {
        yaw_deg = 270 - yaw_deg;
      }
      yaw_deg = yaw_deg + 90;
      msg_ptr->output_msg_ptr->finger_angles.push_back(yaw_deg);
      if (pose_idx == 25 || pose_idx == 29) {
        right_catch_deg += (yaw_deg - 90);
      }
    }
  }
  right_catch_deg /= 2;
  if (right_catch_deg > 110){
    right_catch_deg = 110;
  }
  if (right_catch_deg < 0){
    right_catch_deg = 0;
  }
  float right_finger_distance_mean = right_catch_deg / 110;
  msg_ptr->output_msg_ptr->left_finger_distance = left_finger_distance_mean;
  msg_ptr->output_msg_ptr->right_finger_distance = right_finger_distance_mean;
}

RS_REGISTER_MOTION_CAPTURE(NoitonGloveInertialMotionCapture);

}
}