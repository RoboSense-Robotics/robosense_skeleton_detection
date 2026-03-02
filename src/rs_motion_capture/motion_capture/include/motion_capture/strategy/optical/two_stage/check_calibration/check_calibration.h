//
// Created by sti on 2025/7/17.
//

#ifndef MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_CHECK_CALIBRATION_H
#define MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_CHECK_CALIBRATION_H

#include "motion_capture/common/message.h"

namespace robosense {
namespace motion_capture {

class CheckCalibration{
public:
  using Ptr = std::shared_ptr<CheckCalibration>;
  
  CheckCalibration() = default;
  
  void init(const YAML::Node& cfg_node);

  void process(const Msg::Ptr &msg_ptr);

  std::string name() const { return "CheckCalibration"; }

private:
  void get_pose_frame_points(const std::string& pose_status, const Msg::Ptr &msg_ptr); // 获取一个动作的关键点数据

  void CheckResult();

  std::vector<std::string> pose_status_{
    "stand_attention", // 立正
    // "forward_astandnd_backward", // 前后移动
    // "left_and_right",  // 左右移动
  };   // 人体姿态状态
  std::map<std::string, std::vector<std::vector<float>>> frame_error_dict_; // 帧级别误差
  std::vector<std::vector<float>> world_center_points_; // 世界坐标系下的原点坐标

  int before_frame_end_num_ = 30 * 3; // 执行动作开始前
  int key_frame_end_num_ = 30 * 6;    // 直行动作过程中
  int after_frame_end_num_ = 30 * 7;  // 执行动作结束后
  int cur_frame_ = 0;            // 当前动作采集帧数
  int cur_status_id_ = 0;           // 当前动作序号

  // 误差阈值
  float ac_error_thre_ = 0.085; // 双AC标定误差阈值
  float x_error_thre_ = 0.08;  // x方向误差阈值
  float y_error_thre_ = 0.08;  // y方向误差阈值
  float z_error_thre_ = 0.1;  // z方向误差阈值


  std::vector<int> compute_pose_id_ = {0}; // 只取二维码上的检测点计算双AC误差
  std::vector<std::pair<int, int>> pose_y_connect_id_map_ = {
    {1, 4},
    {2, 5},
    {3, 6},
  };

  std::vector<std::pair<int, int>> pose_z_connect_id_map_ = {
    {1, 2},
    {4, 5},
    {1, 3},
    {4, 6},
  };

  std::string result_save_path_;

  TimeRecorder::Ptr time_recorder_ptr_;
};


} // end motion_capture
} // end robosense

#endif //MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_CHECK_CALIBRATION_H
