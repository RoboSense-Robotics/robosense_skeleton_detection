//
// ROS1 版本 motion_capture 节点
// 对应 ROS2 的 motion_capture_node.cpp
//

#include "production/ros_production.h"
#include "rviz_display/ros_rviz_display.h"
#include "motion_capture/utils/logger/logger.h"
#include <ros/ros.h>
#include <spdlog/spdlog.h>

using namespace robosense::motion_capture;

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_capture");

  ros::NodeHandle nh("~");

  // 读取 ROS 参数 (nh.param 或 ros::param::get)
  bool check_mode, calib_mode;
  std::string collector, config_path, glove_type;
  float ws_center_z;

  nh.param<bool>("check_mode", check_mode, false);
  nh.param<bool>("calib_mode", calib_mode, false);
  nh.param<std::string>("collector", collector, "zed");
  nh.param<std::string>("config", config_path, "config/config.yaml");
  nh.param<std::string>("glove_type", glove_type, "noiton");
  nh.param<float>("ws_center_z", ws_center_z, 1.35f);

  // 初始化全局配置
  std::string config_file = std::string(PROJECT_PATH) + "/" + config_path;
  rally::ConfigureManager::getInstance().setConfigFile(config_file);
  auto& cfg_node = rally::ConfigureManager::getInstance().getCfgNode();
  cfg_node["operator"] = collector;
  cfg_node["check_mode"] = check_mode;
  cfg_node["calib_mode"] = calib_mode;
  cfg_node["glove_type"] = glove_type;
  cfg_node["ws_center_z"] = ws_center_z;

  spdlog::info("Use config path: {}", config_file);
  spdlog::info("Collector: {}", collector);
  spdlog::info("Check mode: {}", check_mode);
  spdlog::info("Calib mode: {}", calib_mode);
  spdlog::info("ws_center_z: {}", ws_center_z);

  // 读取 world center 文件
  std::string world_center_file = std::string(PROJECT_PATH) + "/config/sensor/check_result.yaml";
  if ((!calib_mode) && (!check_mode)) {
    try {
      YAML::Node check_result_info = YAML::LoadFile(world_center_file);
      cfg_node["world_center_x"] = check_result_info["world_center_x"].as<float>();
      cfg_node["world_center_y"] = check_result_info["world_center_y"].as<float>();
      cfg_node["world_center_z"] = check_result_info["world_center_z"].as<float>();
    } catch (...) {
      spdlog::error("read check result failed: {}", world_center_file);
      cfg_node["world_center_x"] = -0.2f;
      cfg_node["world_center_y"] = 0.0f;
      cfg_node["world_center_z"] = 0.6f;
    }
    spdlog::info("world_center_x: {}", cfg_node["world_center_x"]);
    spdlog::info("world_center_y: {}", cfg_node["world_center_y"]);
    spdlog::info("world_center_z: {}", cfg_node["world_center_z"]);
  } else {
    cfg_node["world_center_x"] = 0.0f;
    cfg_node["world_center_y"] = 0.0f;
    cfg_node["world_center_z"] = 0.0f;
  }

  RosProduction::Ptr production_node = std::make_shared<RosProduction>(nh);
  production_node->init();

  if (check_mode) {
    auto check_cb_func = [](const Msg::Ptr& msg_ptr) {
      if (msg_ptr->internal_result_ptr->check_finish) {
        spdlog::info("Check Calibration finished, exiting...");
        ros::shutdown();
      }
    };
    production_node->regOpticalCallback(check_cb_func);
  }

  if (calib_mode) {
    auto calib_cb_func = [](const Msg::Ptr& msg_ptr) {
      if (msg_ptr->internal_result_ptr->calib_finish_map.at(rally::CameraEnum::left_ac_camera) &&
          msg_ptr->internal_result_ptr->calib_finish_map.at(rally::CameraEnum::right_ac_camera)) {
        spdlog::info("Calibration finished, exiting...");
        ros::shutdown();
      }
    };
    production_node->regOpticalCallback(calib_cb_func);
  }

  // 创建 rviz 显示
  bool enable_rviz = cfg_node["ros"]["rviz"]["enable"].as<bool>();
  RosRvizDisplay::Ptr rviz_display = std::make_shared<RosRvizDisplay>(nh);
  if (enable_rviz) {
    rviz_display->init();
    auto display_cb_func = [=](const Msg::Ptr& msg_ptr) { rviz_display->addData(msg_ptr); };
    production_node->regOpticalCallback(display_cb_func);
    rviz_display->start();
  }

  production_node->start();

  ros::spin();
  ros::shutdown();

  return 0;
}
