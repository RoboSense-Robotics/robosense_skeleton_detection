//
// Created by sti on 2025/6/3.
//

#ifndef POSE_DETECTION_SENSOR_MANAGER_H
#define POSE_DETECTION_SENSOR_MANAGER_H

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "rally/common/common.h"
#include "motion_capture/utils/transform/transform.h"
#include "motion_capture/utils/sensor_manager/camera/pinhole_camera.h"
#include "motion_capture/utils/sensor_manager/lidar/lidar.h"

namespace robosense {
namespace motion_capture {

class SensorManager {
public:
  static SensorManager& getInstance() {
    static SensorManager sensor_manager;
    return sensor_manager;
  }

  void init(const YAML::Node& sensor_node);

  void getRectifyMap(rally::CameraEnum type_frame, int dst_width,
                     int dst_height, cv::Mat& map_x, cv::Mat& map_y);

  cv::Mat remap(rally::CameraEnum type_frame, const cv::Mat& map_x,
                const cv::Mat& map_y);

  int getHeight(rally::CameraEnum type_frame);

  int getWidth(rally::CameraEnum type_frame);

  Eigen::Matrix4f getCamera2Lidar(rally::CameraEnum type_frame);

  Eigen::Matrix4f getCamera2World(rally::CameraEnum type_frame);

  cv::Mat getIntrinsic(rally::CameraEnum type_frame);

  cv::Mat getResizeIntrinsic(rally::CameraEnum type_frame, float resize_height,
                             float resize_width);

  cv::Mat getCropResizeIntrinsic(rally::CameraEnum type_frame,
                                 float resize_height,
                                 float resize_width,
                                 float crop_ratio);

  Eigen::Matrix4f getLidar2Camera(rally::LidarEnum type_frame);

  Eigen::Matrix4f getLidar2World(rally::LidarEnum type_frame);

  std::string name() { return "SensorManager"; }

private:
  RALLY_DISALLOW_COPY_AND_ASSIGN(SensorManager)

  SensorManager() = default;

  ~SensorManager() = default;

  void load_camera_params(const YAML::Node& camera_node, CameraCalibOptions& camera_options, rally::CameraEnum camera_type);

  void checkValidCamera(rally::CameraEnum type_frame);

  void checkValidLidar(rally::LidarEnum type_frame);

private:
  std::map<rally::CameraEnum, BaseCamera::Ptr> camera_calib_map_;
  std::map<rally::LidarEnum, BaseLidar::Ptr> lidar_calib_map_;
};

}
}

#endif //POSE_DETECTION_SENSOR_MANAGER_H
