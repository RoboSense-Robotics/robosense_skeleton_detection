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

#include "rally/common/basic_type/odom.h"

namespace rally {

Eigen::Affine3d Odom::toAffine3d() const {
    Eigen::Quaterniond rot;

    rot.w() = orientation_w;
    rot.x() = orientation_x;
    rot.y() = orientation_y;
    rot.z() = orientation_z;

    Eigen::Vector3d trans;

    trans.x() = pose_x;
    trans.y() = pose_y;
    trans.z() = pose_z;

    Eigen::Affine3d transformation;
    transformation.translation() = trans;
    transformation.linear() = rot.toRotationMatrix();

    return transformation;
}

void Odom::fromAffine3d(const Eigen::Affine3d &t) {
    const auto &translation = t.translation();
    pose_x = translation.x();
    pose_y = translation.y();
    pose_z = translation.z();

    Eigen::Quaterniond rot(t.rotation());

    orientation_x = rot.x();
    orientation_y = rot.y();
    orientation_z = rot.z();
    orientation_w = rot.w();
}

Eigen::Matrix4d Pose::toMat() const {
    const Eigen::AngleAxisd init_rotation_x{roll, Eigen::Vector3d::UnitX()};
    const Eigen::AngleAxisd init_rotation_y{pitch, Eigen::Vector3d::UnitY()};
    const Eigen::AngleAxisd init_rotation_z{yaw, Eigen::Vector3d::UnitZ()};

    const Eigen::Translation3d init_translation{x, y, z};

    Eigen::Matrix4d init_matrix{(init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix()};
    return std::move(init_matrix);
}

void Pose::fromMat(const Eigen::Matrix4d &t) {
    x = t(0, 3);
    y = t(1, 3);
    z = t(2, 3);
    roll = std::atan2(t(2, 1), t(2, 2));
    pitch = std::atan2(-t(2, 0), std::sqrt(t(2, 1) * t(2, 1) + t(2, 2) * t(2, 2)));
    yaw = std::atan2(t(1, 0), t(0, 0));
}

}  // namespace rally
