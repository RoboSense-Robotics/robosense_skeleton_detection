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

#include "rally/common/undistort/dynamic_compensation.h"

namespace rally {

void pointsDynamicCompensation(std::vector<Eigen::Vector3d> &corners, double timestamp, const Eigen::Vector3d &vel,
                               const Eigen::Affine3d &g_pose, double undistort_time) {
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

void boxDynamicCompensation(RotatedBox &rotated_box, double timestamp, const Eigen::Vector3d &vel,
                            const Eigen::Affine3d &g_pose, double undistort_time) {
    auto corners = rotated_box.getAllCorners();
    pointsDynamicCompensation(corners, timestamp, vel, g_pose, undistort_time);
    rotated_box.fromCorners(corners);
}

}  // namespace rally
