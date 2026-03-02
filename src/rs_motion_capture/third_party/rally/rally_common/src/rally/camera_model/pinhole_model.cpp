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
#include "rally/common/camera_model/pinhole_model.h"

namespace rally {

PinholeModel::PinholeModel(const PinholeModelOptions &options) {
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

        cv::initUndistortRectifyMap(camera_matrix_, distor_coe_, cv::Mat(), camera_matrix_, image_size_, CV_16SC2,
                                    map_x_,
                                    map_y_);
    }
}

PinholeModel::PinholeModel(const cv::Mat &transform_matrix, const cv::Mat &camera_matrix, const cv::Mat &distor_coe,
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

    cv::initUndistortRectifyMap(camera_matrix_, distor_coe_, cv::Mat(), camera_matrix_, image_size_, CV_16SC2,
                                map_x_,
                                map_y_);
}

void PinholeModel::getProjectPtsDistorted(const std::vector<cv::Point3d> &pts3d, std::vector<cv::Point2i> &pts2d, std::vector<bool> &flag) {
    std::vector<cv::Point2d> tmp_pts2d;
    cv::projectPoints(pts3d, r_vec_, t_vec_, camera_matrix_, distor_coe_, tmp_pts2d);
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

void PinholeModel::getProjectPts(const std::vector<cv::Point3d> &pts3d, std::vector<cv::Point2i> &pts2d,
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
