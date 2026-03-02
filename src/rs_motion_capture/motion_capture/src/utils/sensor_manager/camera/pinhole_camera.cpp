//
// Created by sti on 2025/6/4.
//
#include "motion_capture/utils/sensor_manager/camera/pinhole_camera.h"

namespace robosense {
namespace motion_capture {

void PinholeCamera::initRectifyMap(int dst_width, int dst_height,
                                        cv::Mat &map_x, cv::Mat &map_y) {
  map_x = cv::Mat(height_, width_, CV_32FC2);
  if (cam_model_ == CAMMODEL::PINHOLE) {
    cv::initUndistortRectifyMap(intrinsic_, distortion_, cv::Mat(), intrinsic_,
                                cv::Size(width_, height_), CV_32FC2, map_x,
                                map_y);
  } else if (cam_model_ == CAMMODEL::FISHEYE) {
    cv::Mat map_1(height_, width_, CV_32F);
    cv::Mat map_2(height_, width_, CV_32F);
    cv::fisheye::initUndistortRectifyMap(intrinsic_, distortion_, cv::Mat(),
                                         intrinsic_, cv::Size(width_, height_),
                                         CV_32F, map_1, map_2);
    cv::merge(std::vector<cv::Mat>{map_1, map_2}, map_x);
  } else {
    RTHROW("unknown cam model")
  }
  cv::resize(map_x, map_x, cv::Size(dst_width, dst_height));
}

cv::Mat PinholeCamera::remap(const cv::Mat &map_x, const cv::Mat &map_y) {
  // todo: cpu remap
  return cv::Mat();
}

cv::Mat PinholeCamera::getResizeIntrinsic(float resize_height,
                                               float resize_width) {
  float sw = resize_width / width_;
  float sh = resize_height / height_;
  cv::Mat mat =
          (cv::Mat_<float>(3, 3) << sw, 0.f, 0.f, 0.f, sh, 0.f, 0.f, 0.f, 1.f);
  return mat * intrinsic_;
}

cv::Mat PinholeCamera::getCropResizeIntrinsic(float resize_height,
                                                   float resize_width,
                                                   float crop_ratio) {
  intrinsic_.at<float>(0, 2) -= width_ * (1 - crop_ratio) / 2;
  intrinsic_.at<float>(1, 2) -= height_ * (1 - crop_ratio) / 2;
  float sw = resize_width / (width_ * crop_ratio);
  float sh = resize_height / (height_ * crop_ratio);
  cv::Mat mat =
          (cv::Mat_<float>(3, 3) << sw, 0.f, 0.f, 0.f, sh, 0.f, 0.f, 0.f, 1.f);
  return mat * intrinsic_;
}

}
}
