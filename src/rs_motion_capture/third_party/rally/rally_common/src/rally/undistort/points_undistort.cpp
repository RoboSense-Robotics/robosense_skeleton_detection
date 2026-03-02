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

#include "rally/common/undistort/points_undistort.h"

namespace rally {

Eigen::Affine3d PointsUndistort::staticUndistort(const pcl::PointCloud<RsPoint>::Ptr &cloud_ptr,
                                const Odom &pre_odom, const Odom &cur_odom, double undistort_time) {
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

void PointsUndistort::dynamicUndistort(std::vector<Eigen::Vector3d> &corners,
                      double timestamp,
                      const Eigen::Affine3d &g_pose,
                      const Eigen::Vector3d &vel, double undistort_time) {
    Eigen::Vector3d g_vel;
    {   // transform velocity
        auto tmp_dir = vel;
        double tmp_val = tmp_dir.norm();
        tmp_dir.normalize();

        Eigen::Vector4d tt_pt;
        tt_pt << tmp_dir.head(3), 0.f;
        tt_pt = g_pose.matrix() * tt_pt;

        g_vel = tt_pt.head(3) * tmp_val;
    }
    Eigen::Vector3d delta = g_vel * (timestamp - undistort_time);
    for (size_t i = 0; i < corners.size(); ++i) {
        const auto &corner = corners[i];
        Eigen::Vector4d tmp(corner.x(), corner.y(), corner.z(), 1.);
        tmp = g_pose.matrix() * tmp;
        tmp.head(3) -= delta;
        tmp = g_pose.matrix().inverse() * tmp;
        corners[i] = tmp.head(3);
    }
}

Eigen::Affine3d PointsUndistort::poseInterp(double t, Odom const &aff1, Odom const &aff2) {
    // assume here t1 <= t <= t2
    double t1 = aff1.timestamp;
    double t2 = aff2.timestamp;
    double alpha = 0.0;
    if (t2 != t1) {
        alpha = (t - t1) / (t2 - t1);
    }

    Eigen::Quaternion<double> rot1, rot2;

    rot1.w() = aff1.orientation_w;
    rot1.x() = aff1.orientation_x;
    rot1.y() = aff1.orientation_y;
    rot1.z() = aff1.orientation_z;

    rot2.w() = aff2.orientation_w;
    rot2.x() = aff2.orientation_x;
    rot2.y() = aff2.orientation_y;
    rot2.z() = aff2.orientation_z;

    Eigen::Vector3d trans1, trans2;

    trans1.x() = aff1.pose_x;
    trans1.y() = aff1.pose_y;
    trans1.z() = aff1.pose_z;

    trans2.x() = aff2.pose_x;
    trans2.y() = aff2.pose_y;
    trans2.z() = aff2.pose_z;

    Eigen::Affine3d result;
    result.translation() = (1.0 - alpha) * trans1 + alpha * trans2;
    result.linear() = rot1.slerp(alpha, rot2).toRotationMatrix();

    return result;
}

}  // namespace rally
