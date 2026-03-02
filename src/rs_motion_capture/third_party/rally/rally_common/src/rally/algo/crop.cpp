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

#include <opencv2/opencv.hpp>
#include "rally/common/algo/crop.h"
#include "rally/utils/utils.h"
#include "rally/core/core.h"

namespace rally {

void cropInstanceFromRotatedBox(const pcl::PointCloud<RsPoint>::Ptr &cloud_ptr, const RotatedBox &box,
                                std::vector<int> &instance_idx) {
    instance_idx.clear();
    if (cloud_ptr == nullptr) {
        RWARN << "cropInstanceFromRotatedBox: cropFromBox get cloud is null!";
        return;
    }

    const auto &corners = box.getAllCorners();
    std::vector<cv::Point3d> polygon(4);
    Eigen::Vector3d min_pt(corners[0]), max_pt(corners[0]);
    for (size_t i = 0; i < 4; ++i) {
        polygon[i].x = corners[i].x();
        polygon[i].y = corners[i].y();
        polygon[i].z = corners[i].z();
    }

    for (size_t i = 0; i < 8; ++i) {
        min_pt.x() = std::min(min_pt.x(), corners[i].x());
        min_pt.y() = std::min(min_pt.y(), corners[i].y());
        min_pt.z() = std::min(min_pt.z(), corners[i].z());
        max_pt.x() = std::max(max_pt.x(), corners[i].x());
        max_pt.y() = std::max(max_pt.y(), corners[i].y());
        max_pt.z() = std::max(max_pt.z(), corners[i].z());
    }

    for (size_t i = 0; i < cloud_ptr->size(); ++i) {
        const auto &pt = cloud_ptr->points[i];
        if (std::isnan(pt.x)) {
            continue;
        }
        if (pt.x < min_pt.x() || pt.x > max_pt.x() ||
            pt.y < min_pt.y() || pt.y > max_pt.y() ||
            pt.z < min_pt.z() || pt.z > max_pt.z()) {
            continue;
        }

        if (rally::inPolygon(pt, polygon)) {
            instance_idx.emplace_back(i);
        }
    }
}

}  // namespace rally
