#include "motion_capture/strategy/infrared/tracker/infrared_motion_capture.h"

namespace robosense {
namespace motion_capture {

void InfraredMotionCapture::init(const YAML::Node &cfg_node) {
  AINFO << name() << ": start init...";

  const auto &global_cfg_node =
      rally::ConfigureManager::getInstance().getCfgNode();
  glove_type_ = global_cfg_node["glove_type"].as<std::string>();
  // auto tracker_calib = global_cfg_node["sensor"]["tracker"];
  // auto left_spread_pose = tracker_calib["left_spread_pose"];
  // float left_spread_x = left_spread_pose["x"].as<float>();
  // float left_spread_y = left_spread_pose["y"].as<float>();
  // float left_spread_z = left_spread_pose["z"].as<float>();
  // left_spread_pose_ = {left_spread_x, left_spread_y, left_spread_z};
  // auto right_spread_pose = tracker_calib["right_spread_pose"];
  // float right_spread_x = right_spread_pose["x"].as<float>();
  // float right_spread_y = right_spread_pose["y"].as<float>();
  // float right_spread_z = right_spread_pose["z"].as<float>();
  // right_spread_pose_ = {right_spread_x, right_spread_y, right_spread_z};
  // person_zero_pose_ = {(left_spread_x+right_spread_x) / 2, (left_spread_y + right_spread_y) / 2, (left_spread_z + right_spread_z)/2};

  time_recorder_ptr_ = std::make_shared<TimeRecorder>("InfraredMotionCapture");
}

std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>,
          std::pair<Eigen::Vector3f, Eigen::Quaternionf>>
InfraredMotionCapture::calculateGlovePose(const Msg::Ptr &msg_ptr) {
  Eigen::Vector3f right_position = Eigen::Vector3f::Zero();
  Eigen::Quaternionf right_rotation = Eigen::Quaternionf::Identity();
  Eigen::Vector3f left_position = Eigen::Vector3f::Zero();
  Eigen::Quaternionf left_rotation = Eigen::Quaternionf::Identity();
  if (!msg_ptr->use_glove) {
    AERROR << "Infra-red mode must be fitted with gloves";
  }

  if ("manus" == glove_type_) {
    left_rotation = Eigen::Quaternionf(
        msg_ptr->input_msg_ptr->hand_joints_pose2->data->at(3),
        msg_ptr->input_msg_ptr->hand_joints_pose2->data->at(0),
        msg_ptr->input_msg_ptr->hand_joints_pose2->data->at(1),
        msg_ptr->input_msg_ptr->hand_joints_pose2->data->at(2));
    right_rotation = Eigen::Quaternionf(
        msg_ptr->input_msg_ptr->hand_joints_pose2->data->at(13),
        msg_ptr->input_msg_ptr->hand_joints_pose2->data->at(10),
        msg_ptr->input_msg_ptr->hand_joints_pose2->data->at(11),
        msg_ptr->input_msg_ptr->hand_joints_pose2->data->at(12));
  } else if ("noiton" == glove_type_) {
    const auto &left_pose =
        msg_ptr->input_msg_ptr->hand_joints_pose->data->at(0);
    const auto &right_pose =
        msg_ptr->input_msg_ptr->hand_joints_pose->data->at(20);

    Eigen::Quaternionf right_quat(right_pose.w(), right_pose.x(),
                                  right_pose.y(), right_pose.z());
    // 转换为欧拉角（Z-Y-X，即 yaw-pitch-roll）
    Eigen::Vector3f right_euler =
        right_quat.toRotationMatrix().eulerAngles(2, 1, 0); // yaw, pitch, roll

    // 修改 yaw 和 pitch（绕 Z 和 Y 轴）为反向
    float yaw = right_euler[0];    // 原本绕Z轴的角度
    float pitch = -right_euler[1]; // 原本绕Y轴的角度
    float roll = -right_euler[2];  // 保持绕X轴不变

    // 重新构造四元数：注意顺序是 Z-Y-X
    Eigen::AngleAxisf roll_angle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitch_angle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yaw_angle(yaw, Eigen::Vector3f::UnitZ());

    right_quat = yaw_angle * pitch_angle * roll_angle;

    // // 初始位置x朝下
    Eigen::Quaternionf q_world_rot(
        Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitY()));
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
    Eigen::Quaternionf left_quat(left_pose.w(), left_pose.x(), left_pose.y(),
                                 left_pose.z());
    // 初始位置x朝下
    left_quat = q_world_rot * left_quat;
    Eigen::Matrix3f left_R = (left_quat).toRotationMatrix();
    // x轴替换为z轴
    left_R = left_R * R_local;
    left_rotation = Eigen::Quaternionf(left_R);
    left_rotation.normalize();
  } else {
    AERROR << name() << ": unsupported glove type: " << glove_type_;
  }

  return {{right_position, right_rotation}, {left_position, left_rotation}};
}

std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, std::pair<Eigen::Vector3f, Eigen::Quaternionf>>
  InfraredMotionCapture::passthroughTrackerPose(const Msg::Ptr &msg_ptr){
  auto right_tracker = msg_ptr->input_msg_ptr->right_infrared_tracker;
  auto left_tracker = msg_ptr->input_msg_ptr->left_infrared_tracker;
  Eigen::Vector3f right_position = Eigen::Vector3f::Zero();
  Eigen::Vector3f left_position = Eigen::Vector3f::Zero();
  right_position[0] = right_tracker->data->at(0);
  right_position[1] = right_tracker->data->at(1);
  right_position[2] = right_tracker->data->at(2);

  left_position[0] = left_tracker->data->at(0);
  left_position[1] = left_tracker->data->at(1);
  left_position[2] = left_tracker->data->at(2);

  Eigen::Quaternionf right_rotation = Eigen::Quaternionf(right_tracker->data->at(6),right_tracker->data->at(3),right_tracker->data->at(4),right_tracker->data->at(5));
  Eigen::Quaternionf left_rotation = Eigen::Quaternionf(left_tracker->data->at(6),left_tracker->data->at(3),left_tracker->data->at(4),left_tracker->data->at(5));
  return {{right_position, right_rotation}, {left_position, left_rotation}};
}
void InfraredMotionCapture::process(const Msg::Ptr &msg_ptr) {
  time_recorder_ptr_->tic();
  auto glove_pose = calculateGlovePose(msg_ptr);
  auto tracker_pose = passthroughTrackerPose(msg_ptr);
  msg_ptr->internal_result_ptr->glove_passthrough_pose = glove_pose;
  msg_ptr->output_msg_ptr->end_pose = tracker_pose;
  time_recorder_ptr_->toc();
}

RS_REGISTER_MOTION_CAPTURE(InfraredMotionCapture);

} // namespace motion_capture
} // namespace robosense
