/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "rally/utils/utils.h"
#include "rally/common/camera_model/fisheye_model.h"

namespace rally {

FisheyeSphereCameraModel::FisheyeSphereCameraModel(const FisheyeSphereCameraModelOptions &options) {
    options_ = options;

    assertOptionValid();

    cv::FileStorage fs(options_.config_file, cv::FileStorage::READ);
    if (fs.isOpened()) {
        fs["CameraExtrinsicMat"] >> transform_matrix_;
        fs["CameraMat"] >> camera_matrix_;
        fs["DistCoeff"] >> distor_coe_;
        fs["ImageSize"] >> image_size_;
        fs.release();
    }

    getRemapMat();
    getNewTransformation();
}

FisheyeSphereCameraModel::FisheyeSphereCameraModel(const FisheyeSphereCameraModelOptions &options,
                                                   const cv::Mat &transform_matrix,
                                                   const cv::Mat &camera_matrix, const cv::Mat &distor_coe,
                                                   const cv::Size &image_size) :
options_(options), transform_matrix_(transform_matrix), camera_matrix_(camera_matrix),
distor_coe_(distor_coe), image_size_(image_size) {
    getRemapMat();
    getNewTransformation();
}

void
FisheyeSphereCameraModel::getProjectPts(const std::vector<cv::Point3d> &src_pts, std::vector<cv::Point2i> &dst_pts) {
    dst_pts.clear();

    for (auto pt: src_pts) {
        cv::Vec3d tmp_pt;
        tmp_pt[0] = new_rot_.at<double>(0, 0) * pt.x +
                    new_rot_.at<double>(0, 1) * pt.y +
                    new_rot_.at<double>(0, 2) * pt.z +
                    new_trans_.at<double>(0, 0);
        tmp_pt[1] = new_rot_.at<double>(1, 0) * pt.x +
                    new_rot_.at<double>(1, 1) * pt.y +
                    new_rot_.at<double>(1, 2) * pt.z +
                    new_trans_.at<double>(1, 0);
        tmp_pt[2] = new_rot_.at<double>(2, 0) * pt.x +
                    new_rot_.at<double>(2, 1) * pt.y +
                    new_rot_.at<double>(2, 2) * pt.z +
                    new_trans_.at<double>(2, 0);
        tmp_pt = cv::normalize(tmp_pt);
        double phi = std::asin(tmp_pt[1]);
        double theta = std::atan2(tmp_pt[0], tmp_pt[2]);

        cv::Point2i out_pt(theta * f_ + cx_, phi * f_ + cy_);
        dst_pts.push_back(out_pt);
    }
}

void FisheyeSphereCameraModel::getProjectPts(const std::vector<cv::Point3d> &src_pts, std::vector<cv::Point2i> &dst_pts,
                                             std::vector<bool> &flags) {
    dst_pts.clear();

    for (auto pt: src_pts) {
        cv::Vec3d tmp_pt;
        tmp_pt[0] = new_rot_.at<double>(0, 0) * pt.x +
                    new_rot_.at<double>(0, 1) * pt.y +
                    new_rot_.at<double>(0, 2) * pt.z +
                    new_trans_.at<double>(0, 0);
        tmp_pt[1] = new_rot_.at<double>(1, 0) * pt.x +
                    new_rot_.at<double>(1, 1) * pt.y +
                    new_rot_.at<double>(1, 2) * pt.z +
                    new_trans_.at<double>(1, 0);
        tmp_pt[2] = new_rot_.at<double>(2, 0) * pt.x +
                    new_rot_.at<double>(2, 1) * pt.y +
                    new_rot_.at<double>(2, 2) * pt.z +
                    new_trans_.at<double>(2, 0);
        tmp_pt = cv::normalize(tmp_pt);
        double phi = std::asin(tmp_pt[1]);
        double theta = std::atan2(tmp_pt[0], tmp_pt[2]);

        cv::Point2i out_pt(theta * f_ + cx_, phi * f_ + cy_);
        dst_pts.push_back(out_pt);
        if (tmp_pt[2] < 0) {
            flags.emplace_back(false);
        } else {
            flags.emplace_back(true);
        }
    }
}

void FisheyeSphereCameraModel::getUndistortImage(const cv::Mat &src_img, cv::Mat &dst_img) {
    cv::remap(src_img, dst_img, map_x_, map_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
}

cv::Mat FisheyeSphereCameraModel::eulerAnglesToRotationMatrix(cv::Vec3d &theta, bool is_degree) {
    double roll = theta[0];
    double pitch = theta[1];
    double yaw = theta[2];
    if (is_degree) {
        roll = roll * M_PI / 180.;
        pitch = pitch * M_PI / 180.;
        yaw = yaw * M_PI / 180.;
    }
    // Calculate rotation about x axis
    cv::Mat R_x = (cv::Mat_<double>(3, 3) << std::cos(roll), -std::sin(roll), 0, std::sin(roll), std::cos(
    roll), 0, 0, 0, 1);
    // Calculate rotation about y axis
    cv::Mat R_y = (cv::Mat_<double>(3, 3) << std::cos(pitch), 0, std::sin(pitch), 0, 1, 0, -std::sin(
    pitch), 0, std::cos(pitch));
    // Calculate rotation about z axis
    cv::Mat R_z = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, std::cos(yaw), -std::sin(yaw), 0, std::sin(yaw), std::cos(
    yaw));

    // Combined rotation matrix
    cv::Mat R = R_z * R_y * R_x;

    return R;
}

cv::Vec3d FisheyeSphereCameraModel::rotationMatrixToEulerAngles(cv::Mat &R) {
    double sy = std::sqrt(R.at<double>(0, 0) * R.at<double>(0, 0)
                          + R.at<double>(0, 1) * R.at<double>(0, 1));

    bool singular = sy < 1e-6;

    double x, y, z;
    if (!singular) {
        x = std::atan2(-R.at<double>(0, 1), R.at<double>(0, 0));
        y = std::atan2(R.at<double>(0, 2), sy);
        z = std::atan2(-R.at<double>(1, 2), R.at<double>(2, 2));
    } else {
        x = atan2(R.at<double>(1, 0), R.at<double>(1, 1));
        y = atan2(R.at<double>(0, 2), sy);
        z = 0;
    }
    return cv::Vec3d(x, y, z);
}

void FisheyeSphereCameraModel::getRectMat(cv::Mat& rect_mat) {
    if (!transform_matrix_.data) {
        RTHROW(name() + ": Extrinsic not set!");
    }

    auto rect = transform_matrix_(cv::Rect(0, 0, 3, 3));
    cv::Vec3d euler_angle = rotationMatrixToEulerAngles(rect);
    cv::Vec3d rot1 = {0, euler_angle[1], euler_angle[2]};
    cv::Vec3d rot2 = {0, 0, -90};
    rect_mat = eulerAnglesToRotationMatrix(rot1, false) * eulerAnglesToRotationMatrix(rot2, true);
}

void FisheyeSphereCameraModel::getRemapMat() {
    cv::Mat r;
    getRectMat(r);

    calNewK();

    map_x_ = cv::Mat(options_.dst_size.height, options_.dst_size.width, CV_32FC1, -1);
    map_y_ = cv::Mat(options_.dst_size.height, options_.dst_size.width, CV_32FC1, -1);

    for (int x = 0; x < options_.dst_size.width; x++) {
        for (int y = 0; y < options_.dst_size.height; y++) {
            double theta = (static_cast<double>(x) - cx_) / f_;
            double phi = (static_cast<double>(y) - cy_) / f_;

            double xx = std::sin(theta) * std::cos(phi);
            double yy = std::sin(phi);
            double zz = std::cos(theta) * std::cos(phi);

            double xxc = r.at<double>(0, 0) * xx + r.at<double>(0, 1) * yy + r.at<double>(0, 2) * zz;
            double yyc = r.at<double>(1, 0) * xx + r.at<double>(1, 1) * yy + r.at<double>(1, 2) * zz;
            double zzc = r.at<double>(2, 0) * xx + r.at<double>(2, 1) * yy + r.at<double>(2, 2) * zz;

            if(zzc <= 0){
                xxc = 0.;
                yyc = -options_.dst_size.width;
            } else{
                xxc = xxc / zzc;
                yyc = yyc / zzc;
            }

            double raduis = std::sqrt((xxc * xxc) + (yyc * yyc));
            double distort_theta = std::atan(raduis);
            double undistort_theta = distort_theta + distor_coe_.at<double>(0, 0) * std::pow(distort_theta, 3)
                                     + distor_coe_.at<double>(0, 1) * std::pow(distort_theta, 5)
                                     + distor_coe_.at<double>(0, 2) * std::pow(distort_theta, 7)
                                     + distor_coe_.at<double>(0, 3) * std::pow(distort_theta, 9);

            double xd = (undistort_theta / (raduis + 1.e-6)) * xxc;
            double yd = (undistort_theta / (raduis + 1.e-6)) * yyc;

            map_x_.at<float>(y, x) = camera_matrix_.at<double>(0, 0) * xd + camera_matrix_.at<double>(0, 2);
            map_y_.at<float>(y, x) = camera_matrix_.at<double>(1, 1) * yd + camera_matrix_.at<double>(1, 2);
        }
    }
}

void FisheyeSphereCameraModel::calNewK() {
    f_ = options_.dst_size.width / (options_.dst_h_fov / 180. * M_PI);
    cx_ = options_.dst_size.width / 2.;
    cy_ = options_.dst_size.height - f_ * M_PI_2;
}


FisheyeCameraModel::FisheyeCameraModel(const FisheyeCameraModelOptions &options) {
    options_ = options;
    checkOptionsValid();
    cv::FileStorage fs(options_.config_file, cv::FileStorage::READ);
    if (fs.isOpened()) {
      fs["CameraExtrinsicMat"] >> transform_matrix_;
      fs["CameraMat"] >> camera_matrix_;
      fs["DistCoeff"] >> distor_coe_;
      fs["ImageSize"] >> image_size_;
      fs.release();

      t_vec_ = cv::Mat::zeros(3, 1, CV_64F);
      t_vec_.at<double>(0) = transform_matrix_.at<double>(0, 3);
      t_vec_.at<double>(1) = transform_matrix_.at<double>(1, 3);
      t_vec_.at<double>(2) = transform_matrix_.at<double>(2, 3);

      cv::Mat tmp_mat = cv::Mat::zeros(3, 3, CV_64F);
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
          tmp_mat.at<double>(row, col) = transform_matrix_.at<double>(row, col);
        }
      }
      cv::Rodrigues(tmp_mat, r_vec_);

      cv::fisheye::initUndistortRectifyMap(camera_matrix_, distor_coe_, cv::Mat(), camera_matrix_, image_size_, CV_16SC2,
                                           map_x_,
                                           map_y_);
    }
}

FisheyeCameraModel::FisheyeCameraModel(const cv::Mat &transform_matrix, const cv::Mat &camera_matrix, const cv::Mat &distor_coe,
                                       const cv::Size &image_size) {
    transform_matrix_ = transform_matrix;
    camera_matrix_ = camera_matrix;
    distor_coe_ = distor_coe;
    image_size_ = image_size;

    t_vec_ = cv::Mat::zeros(3, 1, CV_64F);
    t_vec_.at<double>(0) = transform_matrix_.at<double>(0, 3);
    t_vec_.at<double>(1) = transform_matrix_.at<double>(1, 3);
    t_vec_.at<double>(2) = transform_matrix_.at<double>(2, 3);

    cv::Mat tmp_mat = cv::Mat::zeros(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        tmp_mat.at<double>(row, col) = transform_matrix_.at<double>(row, col);
      }
    }
    cv::Rodrigues(tmp_mat, r_vec_);

    cv::fisheye::initUndistortRectifyMap(camera_matrix_, distor_coe_, cv::Mat(), camera_matrix_, image_size_, CV_16SC2,
                                         map_x_,
                                         map_y_);
}

void FisheyeCameraModel::getProjectPtsDistorted(const std::vector<cv::Point3d> &pts3d,
                                                std::vector<cv::Point2i> &pts2d,
                                                std::vector<bool> &flag) {
    std::vector<cv::Point2d> tmp_pts2d;
    cv::fisheye::projectPoints(pts3d, tmp_pts2d, r_vec_, t_vec_, camera_matrix_, distor_coe_);
    pts2d.resize(tmp_pts2d.size());
    for (size_t i = 0; i < tmp_pts2d.size(); ++i) {
      pts2d[i].x = static_cast<int>(tmp_pts2d[i].x);
      pts2d[i].y = static_cast<int>(tmp_pts2d[i].y);
    }

    cv::Mat pc2img;
    cv::Mat camera_matrix_ext;

    cv::Mat in_cloud_mat;
    {
      in_cloud_mat = cv::Mat(4, static_cast<int>(pts3d.size()), CV_64F);
      double *row_x = in_cloud_mat.ptr<double>(0);
      double *row_y = in_cloud_mat.ptr<double>(1);
      double *row_z = in_cloud_mat.ptr<double>(2);
      double *row_w = in_cloud_mat.ptr<double>(3);
      for (std::size_t i = 0; i < pts3d.size(); ++i) {
        *row_x++ = pts3d[i].x;
        *row_y++ = pts3d[i].y;
        *row_z++ = pts3d[i].z;
        *row_w++ = 1;
      }
    }

    cv::hconcat(camera_matrix_, cv::Mat::zeros(3, 1, CV_64FC1), camera_matrix_ext);
    pc2img = camera_matrix_ext * transform_matrix_ * in_cloud_mat;

    for (int i = 0; i < in_cloud_mat.cols; ++i) {
      if (pc2img.at<double>(2, i) < 0) {
        flag.emplace_back(false);
      } else {
        flag.emplace_back(true);
      }
    }
}

void FisheyeCameraModel::getProjectPts(const std::vector<cv::Point3d> &pts3d,
                                       std::vector<cv::Point2i> &pts2d,
                                       std::vector<bool> &flag) {
    cv::Mat pc2img;
    cv::Mat camera_matrix_ext;

    cv::Mat in_cloud_mat;
    {
      in_cloud_mat = cv::Mat(4, static_cast<int>(pts3d.size()), CV_64F);
      double *row_x = in_cloud_mat.ptr<double>(0);
      double *row_y = in_cloud_mat.ptr<double>(1);
      double *row_z = in_cloud_mat.ptr<double>(2);
      double *row_w = in_cloud_mat.ptr<double>(3);
      for (std::size_t i = 0; i < pts3d.size(); ++i) {
        *row_x++ = pts3d[i].x;
        *row_y++ = pts3d[i].y;
        *row_z++ = pts3d[i].z;
        *row_w++ = 1;
      }
    }

    cv::hconcat(camera_matrix_, cv::Mat::zeros(3, 1, CV_64FC1), camera_matrix_ext);
    pc2img = camera_matrix_ext * transform_matrix_ * in_cloud_mat;

    pts2d.clear();
    pts2d.resize(in_cloud_mat.cols);
    for (int i = 0; i < in_cloud_mat.cols; ++i) {
      int u = 0, v = 0;
      u = static_cast<int>(std::round(pc2img.at<double>(0, i) / (std::abs(pc2img.at<double>(2, i)))));
      v = static_cast<int>(std::round(pc2img.at<double>(1, i) / (std::abs(pc2img.at<double>(2, i)))));
      pts2d[i] = cv::Point2d(u, v);
      if (pc2img.at<double>(2, i) < 0) {
        flag.emplace_back(false);
      } else {
        flag.emplace_back(true);
      }
    }
}

}  // namespace rally
