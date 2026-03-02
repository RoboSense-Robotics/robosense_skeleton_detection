//
// Created by sti on 2025/6/6.
//

#include "motion_capture/strategy/optical/two_stage/object_detection/postprocess.h"
#include "motion_capture/utils/sensor_manager/sensor_manager.h"
#include "motion_capture/common/macros.h"
#include "motion_capture/data_structure/point.h"

namespace robosense {
namespace motion_capture {

void ObjectDetectionPostprocess::init(const YAML::Node& cfg_node) {
  AINFO << name() << ": start init...";
  img_attr_.image_width_ = SensorManager::getInstance().getWidth(rally::CameraEnum::left_ac_camera);
  img_attr_.image_height_ = SensorManager::getInstance().getHeight(rally::CameraEnum::left_ac_camera);
  img_attr_.model_input_width_ = 640;
  img_attr_.model_input_height_ = 640;
  img_attr_.resize_scale_ = std::min(static_cast<float>(img_attr_.model_input_width_)/ img_attr_.image_width_,
                                     static_cast<float>(img_attr_.model_input_height_) / img_attr_.image_height_);
  img_attr_.scaled_width_ = static_cast<uint32_t>(img_attr_.image_width_ * img_attr_.resize_scale_);
  img_attr_.scaled_height_ = static_cast<uint32_t>(img_attr_.image_height_ * img_attr_.resize_scale_);
  img_attr_.output_x_offset_ = (img_attr_.model_input_width_ - img_attr_.scaled_width_) / 2;
  img_attr_.output_y_offset_ = (img_attr_.model_input_height_ - img_attr_.scaled_height_) / 2;
  img_attr_.half_width_ = img_attr_.image_width_ / 2;
  img_attr_.half_height_ = img_attr_.image_height_ / 2;

  half_image_data_.resize(img_attr_.half_width_ * img_attr_.half_height_ * 3, 0);

  // 初始化二维码检测参数
#if OPENCV_VERSION_AT_LEAST(4, 8, 0)
  cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  dictionary_ = cv::makePtr<cv::aruco::Dictionary>(dict);
  parameters_ = cv::makePtr<cv::aruco::DetectorParameters>();
#else
  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  parameters_ = cv::aruco::DetectorParameters::create();
#endif

  time_recorder_ptr_ = std::make_shared<TimeRecorder>(name());
  AINFO << name() << ": finish init.";
}

void ObjectDetectionPostprocess::process(const Msg::Ptr &msg_ptr) {
  time_recorder_ptr_->tic();
  AINFO << name() << ": start process...";
  // 获取左右两个AC1检测出来的操作员
  float* left_output = bindings_ptr_->od_bindings_host.output;
  float* right_output = bindings_ptr_->od_bindings_host.output + model_output_dim * model_output_num;
  process_single_image(msg_ptr, left_output, rally::CameraEnum::left_ac_camera);
  process_single_image(msg_ptr, right_output, rally::CameraEnum::right_ac_camera);
  AINFO << name() << ": finish process.";
  time_recorder_ptr_->toc();
}

void ObjectDetectionPostprocess::process_single_image(const Msg::Ptr &msg_ptr, float *output_ptr,
                                                      rally::CameraEnum camera_enum) {
  std::vector<BBox<float>> proposals;
  proposals.reserve(100);
  for (int i = 0; i < model_output_num; ++i) {
    float score = output_ptr[i * model_output_dim + 5];
    float cls = output_ptr[i * model_output_dim + 4];
    if (score < score_thresh || std::fabs(cls - 0.f) > 1e-5) {
      continue;
    }
    proposals.emplace_back(

            output_ptr[i * model_output_dim + 2],
            output_ptr[i * model_output_dim + 0],
            output_ptr[i * model_output_dim + 3],
            output_ptr[i * model_output_dim + 1],
            score                                   // score
    );
  }

  // 如果没有检测到人，标记，返回
  if (proposals.empty()) {
    msg_ptr->internal_result_ptr->is_person_detected_map[camera_enum] = 0;
    last_is_detected_map_[camera_enum] = false;
    return;
  } else {
    msg_ptr->internal_result_ptr->is_person_detected_map[camera_enum] = 1;
  }

  // nms
  std::vector<BBox<float>> person = nms(proposals, nms_iou_thresh);
  // rescale到原始图像
  for (auto& p : person) {
    p.xmin = std::max((p.xmin - img_attr_.output_x_offset_) * img_attr_.image_width_ / (img_attr_.model_input_width_ - 2 * img_attr_.output_x_offset_), 0.f);
    p.ymin = std::max((p.ymin - img_attr_.output_y_offset_) * img_attr_.image_height_ / (img_attr_.model_input_height_ - 2 * img_attr_.output_y_offset_), 0.f);
    p.xmax = std::min((p.xmax - img_attr_.output_x_offset_) * img_attr_.image_width_ / (img_attr_.model_input_width_ - 2 * img_attr_.output_x_offset_), static_cast<float>(img_attr_.image_width_));
    p.ymax = std::min((p.ymax - img_attr_.output_y_offset_) * img_attr_.image_height_ / (img_attr_.model_input_height_ - 2 * img_attr_.output_y_offset_), static_cast<float>(img_attr_.image_height_));
  }
  std::vector<BBox<float>> candidates;
  candidates.reserve(3);

  std::vector<Point2f> candidates_center;

  for (const auto& p : person) {
    // 抠人检测二维码，确定操作员，使用half图像
#ifdef __aarch64__
    cv::Mat half_img = (camera_enum == rally::CameraEnum::left_ac_camera ?
                       cv::Mat(img_attr_.half_height_, img_attr_.half_width_, CV_8UC3, msg_ptr->internal_result_ptr->half_undistorted_image_ptr) :
                       cv::Mat(img_attr_.half_height_, img_attr_.half_width_, CV_8UC3, msg_ptr->internal_result_ptr->half_undistorted_image_ptr + img_attr_.half_width_ * img_attr_.half_height_ * 3));
#else
    if (camera_enum == rally::CameraEnum::left_ac_camera) {
      CUDA_CHECK(cudaMemcpy(
              half_image_data_.data(),
              msg_ptr->internal_result_ptr->half_undistorted_image_ptr,
              half_image_data_.size(),
              cudaMemcpyDeviceToHost
      ));
    } else {
      CUDA_CHECK(cudaMemcpy(
              half_image_data_.data(),
              msg_ptr->internal_result_ptr->half_undistorted_image_ptr + img_attr_.half_width_ * img_attr_.half_height_ * 3,
              half_image_data_.size(),
              cudaMemcpyDeviceToHost
      ));
    }
    cv::Mat half_img(img_attr_.half_height_, img_attr_.half_width_, CV_8UC3, half_image_data_.data());
#endif
    cv::Rect roi(static_cast<int>(p.xmin / 2),
                 static_cast<int>(p.ymin / 2),
                 static_cast<int>((p.xmax - p.xmin) / 2),
                 static_cast<int>((p.ymax - p.ymin) / 2));

    bool isRoiValid = (roi.x >= 0) &&
                      (roi.y >= 0) &&
                      (roi.x + roi.width <= half_img.cols) &&
                      (roi.y + roi.height <= half_img.rows);
    if (!isRoiValid) {
      continue;
    }
    cv::Mat crop_half_img = half_img(roi);
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    cv::aruco::detectMarkers(crop_half_img, dictionary_, corners, ids, parameters_);
    int need_idx = -1;
    for (int i = 0; i < ids.size(); i++){
      if (ids[i] == 0){
        need_idx = i;
        std::vector<cv::Point2f> pts = corners[need_idx];
      	// 计算二维码中心点
      	cv::Point2f center = (pts[0] + pts[1] + pts[2] + pts[3]) / 4.0f;
      	// 将中心点转换到全图坐标系
      	Point2f center_eigen((center.x + roi.x) * 2, (center.y + roi.y) * 2);
      	msg_ptr->internal_result_ptr->qr_code_is_detected_map[camera_enum] = 1;
      	candidates_center.push_back(center_eigen);

      }
    }
    if (need_idx != -1) {
      // 加入候选目标
      candidates.emplace_back(p);
    }
  }

  if (!candidates_center.empty()) {
    // 选择和上一帧最接近的二维码
   // RWARN << name() << " candidate center nums " << candidates_center.size();
    if (last_center_map_.find(camera_enum) != last_center_map_.end()) {
      float min_distance = std::numeric_limits<float>::max();
      int center_idx = -1;
      Point2f last_center = last_center_map_[camera_enum];
      for (size_t i = 0; i < candidates_center.size(); ++i){
        float dx = candidates_center[i].x - last_center.x;
        float dy = candidates_center[i].y - last_center.y;
        float current_distance = (dx * dx + dy * dy);
        if (current_distance < min_distance) {
          center_idx = i;
          min_distance = current_distance;
        }
      }
      msg_ptr->internal_result_ptr->center_map[camera_enum] = candidates_center[center_idx];
    }
    // 从初始化开始没有检出过二维码时,选第一个
    else {
      msg_ptr->internal_result_ptr->center_map[camera_enum] = candidates_center[0];
    }
    last_center_map_[camera_enum] = msg_ptr->internal_result_ptr->center_map[camera_enum];
  }

  


  // 如果候选目标为空，即没有检测到二维码，分两种情况：如果是第一帧,则将图像最中心的人作为最终结果;如果不是第一帧，将和上一帧结果iou最大的目标作为最终结果
  // 如果有多个候选目标，即二维码在多个人身上，分两种情况：如果是第一帧，则将人中心离二维码中心最近的那个目标作为最终结果；如果不是第一帧，将和上一帧结果iou最大的目标作为最终结果
  if (candidates.empty()) {
    if (last_is_detected_map_[camera_enum] = 1) {
      float max_iou = -1.f;
      int idx = -1;
      for (size_t i = 0; i < person.size(); ++i) {
        const auto& candidate = person[i];
        float current_iou = last_bbox_map_[camera_enum].iou(candidate);
        if (current_iou > max_iou) {
          max_iou = current_iou;
          idx = i; // 获取索引
        }
      }
      RENSURE(idx != -1);
      msg_ptr->internal_result_ptr->detected_person_bbox_map[camera_enum] = person[idx];
      last_is_detected_map_[camera_enum] = 1;
      last_bbox_map_[camera_enum] = person[idx];
    } else {
      float distance = std::numeric_limits<float>::max();
      int idx = -1;
      Eigen::Vector2f image_center(img_attr_.image_width_ / 2.0f, img_attr_.image_height_ / 2.0f);
      for (size_t i = 0; i < person.size(); ++i) {
        const auto& p = person[i];
        Eigen::Vector2f person_center((p.xmin + p.xmax) / 2.0f, (p.ymin + p.ymax) / 2.0f);
        float current_distance = (person_center - image_center).norm();
        if (current_distance < distance) {
          distance = current_distance;
          idx = i;
        }
      }
      RENSURE(idx != -1);
      msg_ptr->internal_result_ptr->detected_person_bbox_map[camera_enum] = person[idx];
      last_is_detected_map_[camera_enum] = 1;
      last_bbox_map_[camera_enum] = person[idx];
    }
  } else if (candidates.size() > 1) {
    if (last_is_detected_map_[camera_enum] == 0) {
      float distance = std::numeric_limits<float>::max();
      int idx = -1;
      for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& candidate = candidates[i];
        // 计算人中心和二维码中心的距离
        Point2f person_center((candidate.xmin + candidate.xmax) / 2.0f, (candidate.ymin + candidate.ymax) / 2.0f);
        Point2f qr_center = msg_ptr->internal_result_ptr->center_map[camera_enum];
        float current_distance = person_center.distance(qr_center);
        if (current_distance < distance) {
          distance = current_distance;
          idx = i; // 获取索引
        }
      }
      RENSURE(idx != -1);
      msg_ptr->internal_result_ptr->detected_person_bbox_map[camera_enum] = candidates[idx];
      last_is_detected_map_[camera_enum] = 1;
      last_bbox_map_[camera_enum] = candidates[idx];
    } else {
      float max_iou = -1.f;
      int idx = -1;
      for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& candidate = candidates[i];
        float current_iou = last_bbox_map_[camera_enum].iou(candidate);
        if (current_iou > max_iou) {
          max_iou = current_iou;
          idx = i; // 获取索引
        }
      }
      RENSURE(idx != -1);
      msg_ptr->internal_result_ptr->detected_person_bbox_map[camera_enum] = candidates[idx];
      last_is_detected_map_[camera_enum] = 1;
      last_bbox_map_[camera_enum] = candidates[idx];
    }
  } else {
    msg_ptr->internal_result_ptr->detected_person_bbox_map[camera_enum] = candidates[0];
    last_is_detected_map_[camera_enum] = 1;
    last_bbox_map_[camera_enum] = candidates[0];
  }
}

}
}

