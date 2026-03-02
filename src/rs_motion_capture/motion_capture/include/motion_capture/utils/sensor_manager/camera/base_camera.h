//
// Created by sti on 2025/6/4.
//

#ifndef POSE_DETECTION_BASE_CAMERA_H
#define POSE_DETECTION_BASE_CAMERA_H

#include "rally/common/common.h"

namespace robosense {
namespace motion_capture {

enum class CAMMODEL { PINHOLE, FISHEYE };

struct CameraCalibOptions {
  rally::CameraEnum frame_id;
  CAMMODEL cam_model;
  Eigen::Matrix4f camera_2_lidar;
  Eigen::Matrix4f camera_2_world;
  cv::Mat intrinsic;
  cv::Mat distortion;
  int width;
  int height;
};

class BaseCamera {
public:
  using Ptr = std::shared_ptr<BaseCamera>;

  virtual void initRectifyMap(int dst_width, int dst_height, cv::Mat &map_x, cv::Mat &map_y) = 0;

  virtual cv::Mat remap(const cv::Mat &map_x, const cv::Mat &map_y) = 0;

  virtual cv::Mat getResizeIntrinsic(float resize_height, float resize_width) = 0;

  virtual cv::Mat getCropResizeIntrinsic(float resize_height, float resize_width, float crop_ratio) = 0;

  virtual std::string name() = 0;

public:
  rally::CameraEnum getFrameId() { return frame_id_; }

  Eigen::Matrix4f getCamera2World() { return camera_2_world_; }

  Eigen::Matrix4f getCamera2Lidar() { return camera_2_lidar_; }

  cv::Mat getIntrinsic() { return intrinsic_; }

  cv::Mat getDistortion() { return distortion_; }

  int getWidth() { return width_; }

  int getHeight() { return height_; }


protected:
  rally::CameraEnum frame_id_;
  CAMMODEL cam_model_;
  Eigen::Matrix4f camera_2_world_;
  Eigen::Matrix4f camera_2_lidar_;
  cv::Mat intrinsic_;
  cv::Mat distortion_;
  int width_;
  int height_;
};

}
}

#endif //POSE_DETECTION_BASE_CAMERA_H
