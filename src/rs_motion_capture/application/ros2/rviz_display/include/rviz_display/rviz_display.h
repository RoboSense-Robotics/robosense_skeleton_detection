//
// Created by sti on 2025/6/13.
//

#ifndef POSE_DETECTION_RVIZ_DISPLAY_H
#define POSE_DETECTION_RVIZ_DISPLAY_H

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "motion_capture/interface/pipeline_interface.h"
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "motion_capture/utils/sensor_manager/sensor_manager.h"

namespace robosense {
namespace motion_capture {

class RvizDisplay {
public:
  using Ptr = std::shared_ptr<RvizDisplay>;

  RvizDisplay(const rclcpp::Node::SharedPtr& node_ptr) : node_ptr_(node_ptr)
  {
    worker_.reset(new rally::ConsumerWorker<Msg::Ptr>("rviz_display"));
  }

  ~RvizDisplay() { stop(); }

  void init();

  void display(const Msg::Ptr &msg);

  void start() { worker_->start(); }

  void stop() { worker_->stop(); }

  void addData(const Msg::Ptr &msg_ptr) { worker_->add(msg_ptr); }

private:
  void displayPDResult(const Msg::Ptr& msg_ptr, rally::CameraEnum camera_enum);
  void display3DMarker(const Msg::Ptr& msg_ptr, rally::CameraEnum camera_enum);

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fusion_3d_marker_pub_;

private:
  rclcpp::Node::SharedPtr node_ptr_;
  rally::ConsumerWorker<Msg::Ptr>::Ptr worker_;

  struct Controller {
    bool left_vis_enable = false;
    bool left_pd_enable = false;
    bool right_vis_enable = false;
    bool right_pd_enable = false;
    bool fusion_vis_enable = false;
    bool fusion_3d_marker_enable = false;
  } controller_;

  std::string frame_id_ = "map";

  std::vector<uint8_t> ori_img_data_;

private:
  std::vector<std::pair<int, int>> coco_133_joint_links_ = {
          // 身体主干连接
          {15,13},{13,11},{16,14},{14,12},{11,12},{5,11},{6,12},{5,6},
          {5,7},{6,8},{7,9},{8,10},
          // 头部连接
          {1,2},{0,1},{0,2},{1,3},{2,4},{3,5},{4,6},
          // 脚部连接
          {15,17},{15,18},{15,19},{16,20},{16,21},{16,22},
          // 左手连接
          {91,92},{92,93},{93,94},{94,95},  // 左拇指
          {91,96},{96,97},{97,98},{98,99},  // 左食指
          {91,100},{100,101},{101,102},{102,103},  // 左中指
          {91,104},{104,105},{105,106},{106,107},  // 左无名指
          {91,108},{108,109},{109,110},{110,111},  // 左小指
          // 右手连接
          {112,113},{113,114},{114,115},{115,116},  // 右拇指
          {112,117},{117,118},{118,119},{119,120},  // 右食指
          {112,121},{121,122},{122,123},{123,124},  // 右中指
          {112,125},{125,126},{126,127},{127,128},  // 右无名指
          {112,129},{129,130},{130,131},{131,132}   // 右小指
  };
  std::vector<std::pair<int, int>> halpe_26_joint_links = {
      // 躯干
      {19,11}, {19,12}, {19,18}, {4,6}, {2,4}, {2,0}, {0,1}, {1,3}, {3,5}, {0,18}, {0,17}, {1,2},
      // 手臂
      {5,7}, {7,9}, {6,8}, {8,10}, {6,18}, {5,18},
      // 腿部
      {11,13}, {13,15}, {12,14}, {14,16}, {15,24}, {16,25}, {15,20}, {15,22}, {16,23} ,{16,21}
  };

  std::vector<std::pair<int, int>> line_set_ = {
          {0, 1},
          {1, 2},
          {2, 3},
          {1, 4},
          {0, 4},
          {4, 5},
          {5, 6},
          {0, 9},
          {9, 10},
          {10, 11},
          {0, 12},
          {12, 13},
          {13, 14}
  };
  
  std::vector<int> display_points_idx_ = {0,1,2,3,4,5,6,9,10,11,12,13,14};

  static std_msgs::msg::ColorRGBA createColor(float r, float g, float b, float a = 1.0f) {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  };

  std::map<rally::CameraEnum, std_msgs::msg::ColorRGBA> ac_name_color_map_;

  // dump stuffs
  std::string save_root_path_;
  bool left_pd_dump_;
  std::string left_pd_dir_;
  bool right_pd_dump_;
  std::string right_pd_dir_;

  bool change_coord_ = false;
  float world_x_bias_ = 0.0;
  float world_y_bias_ = 0.0;
  float world_z_bias_ = 0.0;
};

}
}

#endif //POSE_DETECTION_RVIZ_DISPLAY_H
