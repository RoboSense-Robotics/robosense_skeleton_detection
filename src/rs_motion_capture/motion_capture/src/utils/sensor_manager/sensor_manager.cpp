//
// Created by sti on 2025/6/3.
//
#include "motion_capture/utils/sensor_manager/sensor_manager.h"

namespace robosense {
namespace motion_capture {



void SensorManager::init(const YAML::Node& sensor_node) {
  YAML::Node left_ac_node, right_ac_node, calib_node;
  Eigen::Matrix4f left_ac_camera_2_world_matrix = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f right_ac_camera_2_world_matrix = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f left_ac_lidar_2_world_matrix = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f right_ac_lidar_2_world_matrix = Eigen::Matrix4f::Identity();

  // left camera to lidar
  const YAML::Node& camera_left_node = sensor_node["Sensor"]["Camera"];
  Eigen::Quaternionf camera_left_rotation(
          camera_left_node["extrinsic"]["quaternion"]["w"].as<float>(),
          camera_left_node["extrinsic"]["quaternion"]["x"].as<float>(),
          camera_left_node["extrinsic"]["quaternion"]["y"].as<float>(),
          camera_left_node["extrinsic"]["quaternion"]["z"].as<float>());
  Eigen::Vector3f camera_left_translation(
          camera_left_node["extrinsic"]["translation"]["x"].as<float>(),
          camera_left_node["extrinsic"]["translation"]["y"].as<float>(),
          camera_left_node["extrinsic"]["translation"]["z"].as<float>());
  left_ac_camera_2_world_matrix = quaternion_2_matrix(camera_left_rotation, camera_left_translation);

  // right camera to lidar
  const YAML::Node& camera_right_node = sensor_node["Sensor"]["Camera_R"];
  Eigen::Quaternionf camera_right_rotation(
          camera_right_node["extrinsic"]["quaternion"]["w"].as<float>(),
          camera_right_node["extrinsic"]["quaternion"]["x"].as<float>(),
          camera_right_node["extrinsic"]["quaternion"]["y"].as<float>(),
          camera_right_node["extrinsic"]["quaternion"]["z"].as<float>());
  Eigen::Vector3f camera_right_translation(
          camera_right_node["extrinsic"]["translation"]["x"].as<float>(),
          camera_right_node["extrinsic"]["translation"]["y"].as<float>(),
          camera_right_node["extrinsic"]["translation"]["z"].as<float>());
  Eigen::Matrix4f right_ac_camera_2_camera_left_matrix = quaternion_2_matrix(camera_right_rotation, camera_right_translation);

  right_ac_camera_2_world_matrix =  left_ac_camera_2_world_matrix * right_ac_camera_2_camera_left_matrix;



  // 初始化left相机
  rally::CameraEnum camera_left_type = rally::CameraEnum::left_ac_camera;
  CameraCalibOptions camera_left_options;
  load_camera_params(camera_left_node, camera_left_options, camera_left_type);
  camera_left_options.camera_2_world = left_ac_camera_2_world_matrix;
  camera_left_options.camera_2_lidar = left_ac_camera_2_world_matrix;
  camera_calib_map_[camera_left_type].reset(new PinholeCamera(camera_left_options));

  // 初始化left lidar
  rally::LidarEnum lidar_left_type = rally::LidarEnum::left_ac_lidar;
  LidarCalibOptions lidar_left_options;
  lidar_left_options.frame_id = lidar_left_type;
  lidar_left_options.lidar_2_camera = camera_left_options.camera_2_lidar.inverse();
  lidar_left_options.lidar_2_world = left_ac_lidar_2_world_matrix;
  lidar_calib_map_[lidar_left_type].reset(new Lidar(lidar_left_options));


  // 初始化right相机
  rally::CameraEnum camera_right_type = rally::CameraEnum::right_ac_camera;
  CameraCalibOptions camera_right_options;
  load_camera_params(camera_right_node, camera_right_options, camera_right_type);
  camera_right_options.camera_2_world = right_ac_camera_2_world_matrix;
  camera_right_options.camera_2_lidar = right_ac_camera_2_world_matrix;
  camera_calib_map_[camera_right_type].reset(new PinholeCamera(camera_right_options));

  // 初始化right lidar
  rally::LidarEnum lidar_right_type = rally::LidarEnum::right_ac_lidar;
  LidarCalibOptions lidar_right_options;
  lidar_right_options.frame_id = lidar_right_type;
  lidar_right_options.lidar_2_camera = camera_right_options.camera_2_lidar.inverse();
  lidar_right_options.lidar_2_world = right_ac_lidar_2_world_matrix;
  lidar_calib_map_[lidar_right_type].reset(new Lidar(lidar_right_options));
}






// void SensorManager::init(const YAML::Node& sensor_node) {
//   YAML::Node left_ac_node, right_ac_node, calib_node;
//   Eigen::Matrix4f left_ac_camera_2_world_matrix = Eigen::Matrix4f::Identity();
//   Eigen::Matrix4f right_ac_camera_2_world_matrix = Eigen::Matrix4f::Identity();
//   Eigen::Matrix4f left_ac_lidar_2_world_matrix = Eigen::Matrix4f::Identity();
//   Eigen::Matrix4f right_ac_lidar_2_world_matrix = Eigen::Matrix4f::Identity();

//   // 加载多ac标定参数
//   if (sensor_node["calib"].IsDefined()) {
//     rally::yamlSubNode(sensor_node, "calib", calib_node);
//     for (const auto& camera_node : calib_node["Camera"]) {
//       std::string topic = camera_node["topic"].as<std::string>();
//       Eigen::Quaternionf rotation(
//               camera_node["extrinsic"]["quaternion"]["w"].as<float>(),
//               camera_node["extrinsic"]["quaternion"]["x"].as<float>(),
//               camera_node["extrinsic"]["quaternion"]["y"].as<float>(),
//               camera_node["extrinsic"]["quaternion"]["z"].as<float>());
//       Eigen::Vector3f translation(
//               camera_node["extrinsic"]["translation"]["x"].as<float>(),
//               camera_node["extrinsic"]["translation"]["y"].as<float>(),
//               camera_node["extrinsic"]["translation"]["z"].as<float>());
//       if ("/left/rs_camera/rgb" == topic) {
//         left_ac_camera_2_world_matrix = quaternion_2_matrix(rotation, translation);
//       } else if ("/right/rs_camera/rgb" == topic) {
//         right_ac_camera_2_world_matrix = quaternion_2_matrix(rotation, translation);
//       }
//     }
//     for (const auto& lidar_node : calib_node["Lidar"]) {
//       std::string topic = lidar_node["topic"].as<std::string>();
//       Eigen::Quaternionf rotation(
//               lidar_node["extrinsic"]["quaternion"]["w"].as<float>(),
//               lidar_node["extrinsic"]["quaternion"]["x"].as<float>(),
//               lidar_node["extrinsic"]["quaternion"]["y"].as<float>(),
//               lidar_node["extrinsic"]["quaternion"]["z"].as<float>());
//       Eigen::Vector3f translation(
//               lidar_node["extrinsic"]["translation"]["x"].as<float>(),
//               lidar_node["extrinsic"]["translation"]["y"].as<float>(),
//               lidar_node["extrinsic"]["translation"]["z"].as<float>());
//       if ("/left/rs_lidar/points" == topic) {
//         left_ac_lidar_2_world_matrix = quaternion_2_matrix(rotation, translation);
//       } else if ("/right/rs_lidar/points" == topic) {
//         right_ac_lidar_2_world_matrix = quaternion_2_matrix(rotation, translation);
//       }
//     }
//   }


//   if (sensor_node["left_ac"].IsDefined()) {
//     rally::yamlSubNode(sensor_node, "left_ac", left_ac_node);
//     YAML::Node internal_sensor_node, camera_node, lidar_node;
//     rally::yamlSubNode(left_ac_node, "Sensor", internal_sensor_node);
//     rally::yamlSubNode(internal_sensor_node, "Camera", camera_node);
//     rally::yamlSubNode(internal_sensor_node, "Lidar", lidar_node);

//     // 初始化相机
//     rally::CameraEnum camera_type = rally::CameraEnum::left_ac_camera;
//     CameraCalibOptions camera_options;
//     load_camera_params(camera_node, camera_options, camera_type);
//     if (sensor_node["calib"].IsDefined()) {
//       camera_options.camera_2_world = left_ac_camera_2_world_matrix;
//     }
//     camera_calib_map_[camera_type].reset(new PinholeCamera(camera_options));

//     // 初始化lidar
//     rally::LidarEnum lidar_type = rally::LidarEnum::left_ac_lidar;
//     LidarCalibOptions lidar_options;
//     lidar_options.frame_id = lidar_type;
//     lidar_options.lidar_2_camera = camera_options.camera_2_lidar.inverse();
//     if (sensor_node["calib"].IsDefined()) {
//       lidar_options.lidar_2_world = left_ac_lidar_2_world_matrix;
//     }
//     lidar_calib_map_[lidar_type].reset(new Lidar(lidar_options));


//   }
//   if (sensor_node["right_ac"].IsDefined()) {
//     rally::yamlSubNode(sensor_node, "right_ac", right_ac_node);
//     YAML::Node internal_sensor_node, camera_node, lidar_node;
//     rally::yamlSubNode(right_ac_node, "Sensor", internal_sensor_node);
//     rally::yamlSubNode(internal_sensor_node, "Camera", camera_node);
//     rally::yamlSubNode(internal_sensor_node, "Lidar", lidar_node);
//     // 初始化相机
//     rally::CameraEnum camera_type = rally::CameraEnum::right_ac_camera;
//     CameraCalibOptions camera_options;
//     load_camera_params(camera_node, camera_options, camera_type);
//     if (sensor_node["calib"].IsDefined()) {
//       camera_options.camera_2_world = right_ac_camera_2_world_matrix;
//     }
//     camera_calib_map_[camera_type].reset(new PinholeCamera(camera_options));
//     // 初始化lidar
//     rally::LidarEnum lidar_type = rally::LidarEnum::right_ac_lidar;
//     LidarCalibOptions lidar_options;
//     lidar_options.frame_id = lidar_type;
//     lidar_options.lidar_2_camera = camera_options.camera_2_lidar.inverse();
//     if (sensor_node["calib"].IsDefined()) {
//       lidar_options.lidar_2_world = right_ac_lidar_2_world_matrix;
//     }
//     lidar_calib_map_[lidar_type].reset(new Lidar(lidar_options));
//   }
// }

void SensorManager::getRectifyMap(rally::CameraEnum type_frame, int dst_width,
                   int dst_height, cv::Mat& map_x, cv::Mat& map_y) {
  checkValidCamera(type_frame);
  camera_calib_map_.at(type_frame)
          ->initRectifyMap(dst_width, dst_height, map_x, map_y);
}

cv::Mat SensorManager::remap(rally::CameraEnum type_frame, const cv::Mat& map_x,
              const cv::Mat& map_y) {
  checkValidCamera(type_frame);
  return camera_calib_map_.at(type_frame)->remap(map_x, map_y);
}

int SensorManager::getHeight(rally::CameraEnum type_frame) {
  checkValidCamera(type_frame);
  return camera_calib_map_.at(type_frame)->getHeight();
}

int SensorManager::getWidth(rally::CameraEnum type_frame) {
  checkValidCamera(type_frame);
  return camera_calib_map_.at(type_frame)->getWidth();
}

Eigen::Matrix4f SensorManager::getCamera2Lidar(rally::CameraEnum type_frame) {
  checkValidCamera(type_frame);
  return camera_calib_map_.at(type_frame)->getCamera2Lidar();
}

Eigen::Matrix4f SensorManager::getCamera2World(rally::CameraEnum type_frame) {
  checkValidCamera(type_frame);
  return camera_calib_map_.at(type_frame)->getCamera2World();
}

cv::Mat SensorManager::getIntrinsic(rally::CameraEnum type_frame) {
  checkValidCamera(type_frame);
  return camera_calib_map_.at(type_frame)->getIntrinsic();
}

cv::Mat SensorManager::getResizeIntrinsic(rally::CameraEnum type_frame, float resize_height,
                           float resize_width) {
  checkValidCamera(type_frame);
  return camera_calib_map_.at(type_frame)
          ->getResizeIntrinsic(resize_height, resize_width);
}

cv::Mat SensorManager::getCropResizeIntrinsic(rally::CameraEnum type_frame,
                               float resize_height,
                               float resize_width,
                               float crop_ratio) {
  checkValidCamera(type_frame);
  return camera_calib_map_.at(type_frame)
          ->getCropResizeIntrinsic(resize_height, resize_width, crop_ratio);
}

Eigen::Matrix4f SensorManager::getLidar2Camera(rally::LidarEnum type_frame) {
  checkValidLidar(type_frame);
  return lidar_calib_map_.at(type_frame)->getLidar2Camera();
}

Eigen::Matrix4f SensorManager::getLidar2World(rally::LidarEnum type_frame) {
  checkValidLidar(type_frame);
  return lidar_calib_map_.at(type_frame)->getLidar2World();
}

void SensorManager::load_camera_params(const YAML::Node& camera_node, CameraCalibOptions& camera_options, rally::CameraEnum camera_type) {
  camera_options.frame_id = camera_type;
  std::string cam_model = camera_node["intrinsic"]["model"].as<std::string>();
  if (cam_model == "Pinhole") {
    camera_options.cam_model = CAMMODEL::PINHOLE;
  } else {
    RERROR << "Unsupported camera model: " << cam_model;
  }
  Eigen::Quaternionf rotation(camera_node["extrinsic"]["quaternion"]["x"].as<float>(),
                              camera_node["extrinsic"]["quaternion"]["y"].as<float>(),
                              camera_node["extrinsic"]["quaternion"]["z"].as<float>(),
                              camera_node["extrinsic"]["quaternion"]["w"].as<float>());
  Eigen::Vector3f translation(
          camera_node["extrinsic"]["translation"]["x"].as<float>(),
          camera_node["extrinsic"]["translation"]["y"].as<float>(),
          camera_node["extrinsic"]["translation"]["z"].as<float>());
  camera_options.camera_2_lidar = quaternion_2_matrix(rotation, translation);

  std::vector<float> intrinsic = camera_node["intrinsic"]["int_matrix"].as<std::vector<float>>();
  camera_options.intrinsic = cv::Mat(3, 3, CV_32FC1);
  camera_options.intrinsic.at<float>(0, 0) = intrinsic[0];
  camera_options.intrinsic.at<float>(0, 1) = intrinsic[1];
  camera_options.intrinsic.at<float>(0, 2) = intrinsic[2];
  camera_options.intrinsic.at<float>(1, 0) = intrinsic[3];
  camera_options.intrinsic.at<float>(1, 1) = intrinsic[4];
  camera_options.intrinsic.at<float>(1, 2) = intrinsic[5];
  camera_options.intrinsic.at<float>(2, 0) = intrinsic[6];
  camera_options.intrinsic.at<float>(2, 1) = intrinsic[7];
  camera_options.intrinsic.at<float>(2, 2) = intrinsic[8];

  std::vector<float> distortion = camera_node["intrinsic"]["dist_coeff"].as<std::vector<float>>();
  camera_options.distortion = cv::Mat(distortion.size(), 1, CV_32FC1);
  for (size_t i = 0; i < distortion.size(); ++i) {
    camera_options.distortion.at<float>(i, 0) = distortion[i];
  }

  std::vector<int> size = camera_node["intrinsic"]["image_size"].as<std::vector<int>>();
  camera_options.width = size[0];
  camera_options.height = size[1];
}

void SensorManager::checkValidCamera(rally::CameraEnum type_frame) {
  if (camera_calib_map_.find(type_frame) == camera_calib_map_.end()) {
    std::ostringstream os;
    os << name() << ": " << rally::kCameraEnum2NameMap.at(type_frame)
       << " has not been registered";
    RTHROW(os.str());
  }
}

void SensorManager::checkValidLidar(rally::LidarEnum type_frame) {
  if (lidar_calib_map_.find(type_frame) == lidar_calib_map_.end()) {
    std::ostringstream os;
    os << name() << ": " << rally::kLidarEnum2NameMap.at(type_frame)
       << " has not been registered";
    RTHROW(os.str());
  }
}

}
}
