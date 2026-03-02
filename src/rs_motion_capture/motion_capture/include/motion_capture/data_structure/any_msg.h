//
// Created by sti on 2025/6/12.
//

#ifndef MOTION_CAPTURE_DATA_STRUCTURE_ANY_MSG_H
#define MOTION_CAPTURE_DATA_STRUCTURE_ANY_MSG_H

#include "rally/core/core.h"
#include "rally/common/basic_type/odom.h"

namespace robosense {
namespace motion_capture {

enum class MsgType : uint8_t {
  SENSOR = 0,
  IMAGE,
  POINTCLOUD,
  JOINT_POSE,
  JOINT_POSE2,
};

struct AnyMsg {
  using Ptr = std::shared_ptr<AnyMsg>;
  uint64_t getTimestamp() { return timestamp; }
  std::string getFrameID() { return frame_id; }
  MsgType msg_type;
  uint64_t timestamp{0};
  std::string frame_id;
  rally::Odom pose_rel;
  rally::Any::Ptr any_ptr;
};

}
}



#endif //MOTION_CAPTURE_DATA_STRUCTURE_ANY_MSG_H
