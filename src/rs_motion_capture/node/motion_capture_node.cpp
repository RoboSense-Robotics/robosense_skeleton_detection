//
// Created by sti on 2025/6/13.
//

#include "production/ros2_production.h"
#include "rviz_display/rviz_display.h"
#include "motion_capture/utils/logger/logger.h"
#include "rs_log/init.h"

using namespace robosense::motion_capture;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // create node
  rclcpp::Node::SharedPtr node_ptr = rclcpp::Node::make_shared("motion_capture");

  // read param
  node_ptr->declare_parameter<bool>("check_mode", false);
  node_ptr->declare_parameter<bool>("calib_mode", false);
  node_ptr->declare_parameter<std::string>("collector", "zed");
  node_ptr->declare_parameter<std::string>("config", "config/config.yaml");
  node_ptr->declare_parameter<std::string>("glove_type", "noiton");
  node_ptr->declare_parameter<float>("ws_center_z", 1.35);
  bool check_mode = node_ptr->get_parameter("check_mode").as_bool();
  bool calib_mode = node_ptr->get_parameter("calib_mode").as_bool();
  std::string collector = node_ptr->get_parameter("collector").as_string();
  std::string config_path = node_ptr->get_parameter("config").as_string();
  std::string glove_type = node_ptr->get_parameter("glove_type").as_string();
  float ws_center_z = static_cast<float>(node_ptr->get_parameter("ws_center_z").as_double());

  // init global config manager
  std::string config_file = std::string(PROJECT_PATH) + "/" + config_path;
  rally::ConfigureManager::getInstance().setConfigFile(config_file);
  auto& cfg_node = rally::ConfigureManager::getInstance().getCfgNode();
  cfg_node["operator"] = collector;
  if (check_mode) {
    cfg_node["check_mode"] = true;
  } else {
    cfg_node["check_mode"] = false;
  }
  if (calib_mode) {
    cfg_node["calib_mode"] = true;
  } else {
    cfg_node["calib_mode"] = false;
  }
  cfg_node["glove_type"] = glove_type;
  cfg_node["ws_center_z"] = ws_center_z;

  // set log level and log dir
  /*setLogger(cfg_node["log"]["log_to_dir"].as<std::string>(),*/
  /*          cfg_node["log"]["level"].as<std::string>(),*/
  /*          "async");*/
  
  robosense::log::Init("motion_capture");
  AINFO << "Use config path: " << config_file;
  AINFO << "Collector: " << collector;
  AINFO << "Check mode: " << check_mode;
  AINFO << "Calib mode: " << calib_mode;
  AINFO << "ws_center_z: " << ws_center_z;

  // read world center file to cfg
  std::string world_center_file = std::string(PROJECT_PATH) + "/config/sensor/check_result.yaml";
  if ((!cfg_node["calib_mode"].as<bool>()) && (!cfg_node["check_mode"].as<bool>())) {
    try {
      YAML::Node check_result_info = YAML::LoadFile(world_center_file);
      cfg_node["world_center_x"] = check_result_info["world_center_x"].as<float>();
      cfg_node["world_center_y"] = check_result_info["world_center_y"].as<float>();
      cfg_node["world_center_z"] = check_result_info["world_center_z"].as<float>();
    } catch (...) {
      AERROR << "read check result failed: " << world_center_file;
      cfg_node["world_center_x"] = -0.2;
      cfg_node["world_center_y"] = 0.0;
      cfg_node["world_center_z"] = 0.6;
    }
    AINFO << "world_center_x: " << cfg_node["world_center_x"];
    AINFO << "world_center_y: " << cfg_node["world_center_y"];
    AINFO << "world_center_z: " << cfg_node["world_center_z"];
  } else {
    cfg_node["world_center_x"] = -0.0;
    cfg_node["world_center_y"] = 0.0;
    cfg_node["world_center_z"] = 0.0;
  }

  Ros2Production::Ptr production_node = std::make_shared<Ros2Production>(node_ptr);
  production_node->init();
  
  if (check_mode) {
    auto check_cb_func = [](const Msg::Ptr &msg_ptr) {
        if (msg_ptr->internal_result_ptr->check_finish) {
          AINFO << "Check Calibration finished, exiting...";
          rclcpp::shutdown();
        }
    };
    production_node->regOpticalCallback(check_cb_func);
  }

  if (calib_mode) {
    auto calib_cb_func = [](const Msg::Ptr &msg_ptr) {
        if (msg_ptr->internal_result_ptr->calib_finish_map.at(rally::CameraEnum::left_ac_camera) &&
            msg_ptr->internal_result_ptr->calib_finish_map.at(rally::CameraEnum::right_ac_camera)) {
          AINFO << "Calibration finished, exiting...";
          rclcpp::shutdown();
        }
    };
    production_node->regOpticalCallback(calib_cb_func);
  }

  // create rviz display
  bool enable_rviz = cfg_node["ros"]["rviz"]["enable"].as<bool>();
  RvizDisplay::Ptr rviz_display = std::make_shared<RvizDisplay>(node_ptr);
  if (enable_rviz) {
    rviz_display->init();
    auto display_cb_func = [=](const Msg::Ptr &msg_ptr) {
        rviz_display->addData(msg_ptr);
    };
    production_node->regOpticalCallback(display_cb_func);
    rviz_display->start();
  }

  production_node->start();

  rclcpp::spin(node_ptr);
  rclcpp::shutdown();

  return 0;
}
