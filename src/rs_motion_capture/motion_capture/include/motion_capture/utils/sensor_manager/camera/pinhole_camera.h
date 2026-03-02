//
// Created by sti on 2025/6/4.
//

#ifndef POSE_DETECTION_PINHOLE_CAMERA_H
#define POSE_DETECTION_PINHOLE_CAMERA_H

#include "motion_capture/utils/sensor_manager/camera/base_camera.h"

namespace robosense {
namespace motion_capture {

class PinholeCamera : public BaseCamera {
public:
  using Ptr = std::shared_ptr<PinholeCamera>;

  PinholeCamera() = delete;

  PinholeCamera(const CameraCalibOptions &options) {
    frame_id_ = options.frame_id;
    cam_model_ = options.cam_model;
    camera_2_lidar_ = options.camera_2_lidar;
    camera_2_world_ = options.camera_2_world;
    intrinsic_ = options.intrinsic;
    distortion_ = options.distortion;
    width_ = options.width;
    height_ = options.height;
  }

  void initRectifyMap(int dst_width, int dst_height, cv::Mat &map_x, cv::Mat &map_y) override;

  cv::Mat remap(const cv::Mat &map_x, const cv::Mat &map_y) override;

  cv::Mat getResizeIntrinsic(float resize_height, float resize_width) override;

  cv::Mat getCropResizeIntrinsic(float resize_height, float resize_width, float crop_ratio) override;

  std::string name() override { return "PinholeCamera"; }
};

}
}

#endif //POSE_DETECTION_PINHOLE_CAMERA_H
