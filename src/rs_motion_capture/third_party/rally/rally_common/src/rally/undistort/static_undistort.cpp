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

#include <pcl/common/transforms.h>
#include "rally/common/undistort/static_unditort.h"
#include "rally/common/basic_type/rotated_box_info.h"
#include "rally/common/basic_ops/basic_ops.h"
#include "rally/common/algo/algo.h"

namespace rally {

Eigen::Affine3d pointsStaticUndistort(const pcl::PointCloud<RsPoint>::Ptr &cloud_ptr,
                                      const Odom &pre_odom,
                                      const Odom &cur_odom,
                                      double undistort_time) {
    RENSURE(pre_odom.timestamp < cur_odom.timestamp);

    auto dst_pose = poseInterp(undistort_time, pre_odom, cur_odom);

    for (size_t i = 0; i < cloud_ptr->size(); ++i) {
        auto &pt = cloud_ptr->points[i];
        auto pt_pose = poseInterp(pt.timestamp, pre_odom, cur_odom);
        auto t_mat = dst_pose.matrix().inverse() * pt_pose.matrix();

        Eigen::Affine3d transformation;
        transformation.matrix() = t_mat;
        Eigen::Vector3d pt1(pt.x, pt.y, pt.z);
        pcl::transformPoint(pt1, pt1, transformation);
        pt.x = pt1.x();
        pt.y = pt1.y();
        pt.z = pt1.z();
    }

    return dst_pose;
}

void boxStaticCompensation(const pcl::PointCloud<RsPoint>::Ptr &cloud_ptr,
                           const Odom &pre_odom,
                           const Odom &cur_odom,
                           const std::vector<int> &instance_idx,
                           const RotatedBox &in_box,
                           const double &compensate_time,
                           RotatedBox &out_box) {
    out_box = in_box;
    double min_z = in_box.getCenter().z() - 0.5 * in_box.getSize().z();
    double max_z = in_box.getCenter().z() + 0.5 * in_box.getSize().z();
    if (instance_idx.empty() || cloud_ptr == nullptr || cloud_ptr->empty()) {
        return;
    }

    pcl::PointCloud<RsPoint>::Ptr box_cloud_ptr(new pcl::PointCloud <RsPoint>);
    std::array<int, 4> geo_pt_to_corner_pt;
    {
        const auto &geo_box_info = getBoxInfos(in_box);
        Eigen::Vector2d major_pt1, major_pt2, pre_pt1, pre_pt2, nex_pt1, nex_pt2;
        std::pair<int, int> pre_pair;
        std::pair<int, int> nex_pair;
        if (geo_box_info.type == RotatedBboxType::TWO_PTS_FACE) {
            const auto &corners = geo_box_info.corners;
            const auto &major_corners = geo_box_info.two_face_infos.face_line_pair;
            pre_pair = geo_box_info.pre_along_line_pair;
            nex_pair = geo_box_info.nex_along_line_pair;
            major_pt1 = corners[major_corners.first];
            major_pt2 = corners[major_corners.second];
            pre_pt1 = corners[pre_pair.first];
            pre_pt2 = corners[pre_pair.second];
            nex_pt1 = corners[nex_pair.first];
            nex_pt2 = corners[nex_pair.second];
        } else {
            const auto &corners = geo_box_info.corners;
            const auto &major_corners = geo_box_info.three_face_infos.major_dir_line_pair;
            pre_pair = geo_box_info.pre_along_line_pair;
            nex_pair = geo_box_info.nex_along_line_pair;
            major_pt1 = corners[major_corners.first];
            major_pt2 = corners[major_corners.second];
            pre_pt1 = corners[pre_pair.first];
            pre_pt2 = corners[pre_pair.second];
            nex_pt1 = corners[nex_pair.first];
            nex_pt2 = corners[nex_pair.second];
        }

        geo_pt_to_corner_pt[0] = pre_pair.second;
        geo_pt_to_corner_pt[1] = pre_pair.first;
        geo_pt_to_corner_pt[2] = nex_pair.first;
        geo_pt_to_corner_pt[3] = nex_pair.second;

        {
            int major_min_dis_pt_idx{-1}, pre_min_dis_pt_idx{-1}, nex_min_dis_pt_idx{-1};
            double major_min_dis = std::numeric_limits<double>::max();
            double pre_min_dis = std::numeric_limits<double>::max();
            double nex_min_dis = std::numeric_limits<double>::max();
            for (size_t k = 0; k < instance_idx.size(); ++k) {
                const auto &pt_idx = instance_idx[k];
                const auto &pt = cloud_ptr->points[pt_idx];
                Eigen::Vector2d pt_e(pt.x, pt.y);
                {
                    const auto &pt_to_line_dis = pointToLine(pt_e, major_pt1, major_pt2);
                    if (major_min_dis > pt_to_line_dis) {
                        major_min_dis = pt_to_line_dis;
                        major_min_dis_pt_idx = pt_idx;
                    }
                }
                {
                    const auto &pt_to_line_dis = pointToLine(pt_e, pre_pt1, pre_pt2);
                    if (pre_min_dis > pt_to_line_dis) {
                        pre_min_dis = pt_to_line_dis;
                        pre_min_dis_pt_idx = pt_idx;
                    }
                }
                {
                    const auto &pt_to_line_dis = pointToLine(pt_e, nex_pt1, nex_pt2);
                    if (nex_min_dis > pt_to_line_dis) {
                        nex_min_dis = pt_to_line_dis;
                        nex_min_dis_pt_idx = pt_idx;
                    }
                }
            }

            RsPoint pt;
            pt.x = 0;
            pt.y = 0;
            pt.z = 0;
            {
                pt.x = major_pt1.x();
                pt.y = major_pt1.y();
                pt.timestamp = cloud_ptr->points[major_min_dis_pt_idx].timestamp;
                box_cloud_ptr->points.emplace_back(pt);

                pt.x = major_pt2.x();
                pt.y = major_pt2.y();
                box_cloud_ptr->points.emplace_back(pt);
            }
            {
                pt.x = pre_pt2.x();
                pt.y = pre_pt2.y();
                pt.timestamp = cloud_ptr->points[pre_min_dis_pt_idx].timestamp;
                box_cloud_ptr->points.emplace_back(pt);
            }
            {
                pt.x = nex_pt2.x();
                pt.y = nex_pt2.y();
                pt.timestamp = cloud_ptr->points[nex_min_dis_pt_idx].timestamp;
                box_cloud_ptr->points.emplace_back(pt);
            }
        }
    }

    pointsStaticUndistort(box_cloud_ptr, pre_odom, cur_odom, compensate_time);

    std::vector<Eigen::Vector3d> corners(8);
    const auto &major_pt1 = box_cloud_ptr->points[0];
    const auto &major_pt2 = box_cloud_ptr->points[1];
    const auto &pre_pt = box_cloud_ptr->points[2];
    const auto &nex_pt = box_cloud_ptr->points[3];

    Eigen::Vector4d line_pt(major_pt1.x, major_pt1.y, 0, 0);
    Eigen::Vector4d line_dir(major_pt2.x - major_pt1.x, major_pt2.y - major_pt1.y, 0, 0);

    {
        Eigen::Vector4d pt(pre_pt.x, pre_pt.y, 0, 0);

        // double k = (DOT_PROD_3D (points[i], p21) - dotA_B) / dotB_B;
        double k = (pt.dot(line_dir) - line_pt.dot(line_dir)) / line_dir.dot(line_dir);

        Eigen::Vector4d pp = line_pt + k * line_dir;

        auto idx = geo_pt_to_corner_pt[1];
        corners[idx].x() = pp.x();
        corners[idx].y() = pp.y();
        corners[idx].z() = min_z;

        corners[idx + 4].x() = pp.x();
        corners[idx + 4].y() = pp.y();
        corners[idx + 4].z() = max_z;
    }
    {
        Eigen::Vector4d pt(nex_pt.x, nex_pt.y, 0, 0);

        // double k = (DOT_PROD_3D (points[i], p21) - dotA_B) / dotB_B;
        double k = (pt.dot(line_dir) - line_pt.dot(line_dir)) / line_dir.dot(line_dir);

        Eigen::Vector4d pp = line_pt + k * line_dir;

        auto idx = geo_pt_to_corner_pt[2];
        corners[idx].x() = pp.x();
        corners[idx].y() = pp.y();
        corners[idx].z() = min_z;

        corners[idx + 4].x() = pp.x();
        corners[idx + 4].y() = pp.y();
        corners[idx + 4].z() = max_z;
    }
    {
        auto idx = geo_pt_to_corner_pt[3];
        corners[idx].x() = nex_pt.x;
        corners[idx].y() = nex_pt.y;
        corners[idx].z() = min_z;

        corners[idx + 4].x() = nex_pt.x;
        corners[idx + 4].y() = nex_pt.y;
        corners[idx + 4].z() = max_z;
    }
    {
        auto idx = geo_pt_to_corner_pt[0];
        corners[idx].x() = pre_pt.x;
        corners[idx].y() = pre_pt.y;
        corners[idx].z() = min_z;

        corners[idx + 4].x() = pre_pt.x;
        corners[idx + 4].y() = pre_pt.y;
        corners[idx + 4].z() = max_z;
    }
    out_box.fromCorners(corners);
}

}  // namespace rally
