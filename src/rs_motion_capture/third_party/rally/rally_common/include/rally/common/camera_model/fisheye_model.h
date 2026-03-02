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
#ifndef RALLY_COMMON_CAMERA_MODEL_FISHEYE_MODEL_H
#define RALLY_COMMON_CAMERA_MODEL_FISHEYE_MODEL_H

#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>

namespace rally {

struct FisheyeSphereCameraModelOptions {
    std::string config_file;
    cv::Size dst_size = cv::Size(1920, 1408);
    double dst_h_fov = 180;
    double src_v_fov = 154;
};

class FisheyeSphereCameraModel {
public:
    using Ptr = std::shared_ptr<FisheyeSphereCameraModel>;

    FisheyeSphereCameraModel(const FisheyeSphereCameraModelOptions &options);

    FisheyeSphereCameraModel(const FisheyeSphereCameraModelOptions &options, const cv::Mat &transform_matrix,
                             const cv::Mat &camera_matrix, const cv::Mat &distor_coe,
                             const cv::Size &image_size);

    cv::Mat getTransformMatrix() const {
        return transform_matrix_;
    }

    cv::Size getSize() const {
        return image_size_;
    }

    cv::Size getUndistortedSize() const {
        return options_.dst_size;
    }

    cv::Mat getIntriMatrix() const {
        return camera_matrix_;
    }

    cv::Mat getDistort() const {
        return distor_coe_;
    }

    cv::Mat getNewRot() const {
        return new_rot_;
    }

    cv::Mat getNewTrans() const {
        return new_trans_;
    }

    void getProjectPts(const std::vector<cv::Point3d> &src_pts, std::vector<cv::Point2i> &dst_pts);

    void getProjectPts(const std::vector<cv::Point3d> &src_pts, std::vector<cv::Point2i> &dst_pts,
                       std::vector<bool> &flags);

    void getUndistortImage(const cv::Mat &src_img, cv::Mat &dst_img);

private:
    void assertOptionValid() {
        // todo
    }

    std::string name() {
        return "FisheyeSphereCameraModel";
    }

    cv::Mat eulerAnglesToRotationMatrix(cv::Vec3d &theta, bool is_degree = true);

    cv::Vec3d rotationMatrixToEulerAngles(cv::Mat &R);

//    void getRollPitch(double &roll, double &pitch);
    void getRectMat(cv::Mat& rect_mat);

    void getRemapMat();

    void getNewTransformation() {
        cv::Mat r;
        getRectMat(r);

        new_rot_ = r.inv() * transform_matrix_(cv::Rect(0, 0, 3, 3));
        new_trans_ = r.inv() * transform_matrix_(cv::Rect(3, 0, 1, 3));
    }

    void calNewK();

    FisheyeSphereCameraModelOptions options_;

    double f_;  // focal length of output image
    double cx_;  // x coordinate of output image center
    double cy_;  // y coordinate of output image center

    cv::Mat map_x_;
    cv::Mat map_y_;

    // mono camera model
    cv::Mat transform_matrix_;  // Rt
    cv::Mat camera_matrix_;  // K
    cv::Mat distor_coe_;     // D

    cv::Size image_size_;

    // new rot trans
    cv::Mat new_rot_;
    cv::Mat new_trans_;
};

struct FisheyeCameraModelOptions {
  std::string config_file;
  cv::Size dst_size = cv::Size(1920, 1536);
};

class FisheyeCameraModel {
public:
    using Ptr = std::shared_ptr<FisheyeCameraModel>;

    FisheyeCameraModel(const FisheyeCameraModelOptions &options);

    FisheyeCameraModel(const cv::Mat &transform_matrix, const cv::Mat &camera_matrix, const cv::Mat &distor_coe,
                       const cv::Size &image_size);
    cv::Mat getTransformMatrix() const {
      return transform_matrix_;
    }

    cv::Size getSize() const {
      return image_size_;
    }

    cv::Size getUndistortedSize() const {
      return options_.dst_size;
    }

    cv::Mat getIntriMatrix() const {
      return camera_matrix_;
    }

    cv::Mat getDistort() const {
      return distor_coe_;
    }

    void undistortImage(const cv::Mat &raw_img, cv::Mat &undistorted_img) {
      cv::remap(raw_img, undistorted_img, map_x_, map_y_, CV_INTER_LINEAR);
    }

    void getProjectPts(const std::vector<cv::Point3d> &pts3d, std::vector<cv::Point2i> &pts2d, std::vector<bool> &flag);

    void getProjectPtsDistorted(const std::vector<cv::Point3d> &pts3d, std::vector<cv::Point2i> &pts2d,
                                std::vector<bool> &flag);

private:
    std::string name() {
      return "PinholeModel";
    }

    void checkOptionsValid() {
      // todo
    }

    FisheyeCameraModelOptions options_;

    cv::Mat map_x_;
    cv::Mat map_y_;
    cv::Size image_size_;
    // mono camera model
    cv::Mat camera_matrix_;
    cv::Mat distor_coe_;
    cv::Mat transform_matrix_;
    // cv version
    cv::Mat r_vec_;
    cv::Mat t_vec_;
};

}  // namespace rally

#endif  // RALLY_COMMON_CAMERA_MODEL_FISHEYE_MODEL_H
