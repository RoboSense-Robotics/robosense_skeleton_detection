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
#ifndef RALLY_COMMON_CAMERA_MODEL_PINHOLE_MODEL_H
#define RALLY_COMMON_CAMERA_MODEL_PINHOLE_MODEL_H

#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc_c.h>

namespace rally {

struct PinholeModelOptions {
    std::string config_file;
};

class PinholeModel {
public:
    using Ptr = std::shared_ptr<PinholeModel>;

    PinholeModel(const PinholeModelOptions &options);

    PinholeModel(const cv::Mat &transform_matrix, const cv::Mat &camera_matrix, const cv::Mat &distor_coe,
                 const cv::Size &image_size);

    void undistortImage(const cv::Mat &raw_img, cv::Mat &undistorted_img) {
        cv::remap(raw_img, undistorted_img, map_x_, map_y_, CV_INTER_LINEAR);
    }

    cv::Mat getTransformMatrix() const {
        return transform_matrix_;
    }

    cv::Size getSize() const {
        return image_size_;
    }

    cv::Mat getIntriMatrix() const {
        return camera_matrix_;
    }

    cv::Mat getDistort() const {
        return distor_coe_;
    }

    cv::Mat getMapX() const {
      return map_x_;
    }

    cv::Mat getMapY() const {
      return map_y_;
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

    PinholeModelOptions options_;

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

#endif  // RALLY_COMMON_CAMERA_MODEL_PINHOLE_MODEL_H
