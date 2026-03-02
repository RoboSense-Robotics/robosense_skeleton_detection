//
// Created by sti on 2025/6/12.
//

#ifndef MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_BONE_CALIBRATION_BONE_CALIBRATION_H
#define MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_BONE_CALIBRATION_BONE_CALIBRATION_H

#include "motion_capture/common/message.h"

namespace robosense {
namespace motion_capture {

class BoneCalibration {
public:
  using Ptr = std::shared_ptr<BoneCalibration>;

  void init();

  void process(const Msg::Ptr& msg_ptr);

  void calibSingleAC(const Msg::Ptr& msg_ptr, rally::CameraEnum camera_enum);

  std::string name() const { return "BoneCalibration"; }

private:
  std::vector<std::pair<int, int>> opt_line_set_ = {
          {0, 1},
          {1, 2},
          {2, 3},
          {0, 4},
          {4, 5},
          {5, 6},
          {3, 7},
          {6, 8},
          {0, 9},
          {9, 10},
          {10, 11},
          {0, 12},
          {12, 13},
          {13, 14}
  };

  std::map<rally::CameraEnum, std::string> calibration_file_save_path_map_;
  std::map<rally::CameraEnum, uint32_t> calib_cnt_map_;
  std::map<rally::CameraEnum, uint32_t> valid_calib_cnt_map_;
  std::map<rally::CameraEnum, std::array<float, 14>> calib_bone_sum_len_map_;
  std::array<float, 14> calib_bone_default_;
  uint32_t calib_max_cnt_ = 400;

  uint32_t cur_abnormal_time_ = 0;
  uint32_t info_hz_ = 200; // 每200帧异常结果更新一帧calib_info
};

}
}

#endif //MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_BONE_CALIBRATION_BONE_CALIBRATION_H
