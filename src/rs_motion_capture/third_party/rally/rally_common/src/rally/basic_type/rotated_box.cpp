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

#include "rally/common/basic_type/rotated_box.h"

namespace rally {

RotatedBox::RotatedBox() {
    center_ = Eigen::Vector3d(0., 0., 0.);
    size_ = Eigen::Vector3d(0., 0., 0.);
    yaw_ = 0.;
}

RotatedBox::RotatedBox(const Eigen::Vector3d &center, const Eigen::Vector3d &size, float yaw) {
    center_ = center;
    size_ = size;
    yaw_ = yaw;
}

Eigen::Vector3d RotatedBox::getDirection() const {
    Eigen::Vector3d dir = Eigen::Vector3d(std::cos(yaw_), std::sin(yaw_), 0);
    dir.z() = 0.;
    dir.normalize();
    return dir;
}

void RotatedBox::checkBox() {
    const auto &corners = getAllCorners();
    auto l1 = (corners[0] - corners[1]).norm();
    auto l2 = (corners[1] - corners[2]).norm();
    if (l1 > l2) {
        return;
    }
    size_.x() = l2;
    size_.y() = l1;
    Eigen::Vector3d dir = corners[1] - corners[2];
    yaw_ = std::atan2(dir.y(), dir.x());
}

void RotatedBox::fixDirection(const Eigen::Vector3d &vel) {
    Eigen::Vector2d dir_2d = Eigen::Vector2d(std::cos(yaw_), std::sin(yaw_));
    auto vel_2d = vel.head(2);

    double cos_val = dir_2d.dot(vel_2d) / (dir_2d.norm() * vel_2d.norm());
    double angle_rad = std::acos(cos_val);

    while (angle_rad < -M_PI) {
        angle_rad += 2 * M_PI;
    }
    while (angle_rad > M_PI) {
        angle_rad += 2 * M_PI;
    }

    if (std::abs(angle_rad) > M_PI / 2.) {
        yaw_ += M_PI;
        if (yaw_ > M_PI) {
            yaw_ -= M_PI * 2;
        }
    }
}

std::vector<Eigen::Vector3d> RotatedBox::getAllCorners() const {
    std::vector<Eigen::Vector3d> corners;
    corners.resize(8, Eigen::Vector3d::Zero());

    Eigen::Vector3d dir = getDirection();
    Eigen::Vector3d ortho_dir = Eigen::Vector3d(-dir.y(), dir.x(), 0);
    Eigen::Vector3d z_dir = Eigen::Vector3d(0, 0, 1);

    corners[0] = center_ + dir * size_.x() * 0.5 + ortho_dir * size_.y() * 0.5 - z_dir * size_.z() * 0.5;
    corners[1] = center_ - dir * size_.x() * 0.5 + ortho_dir * size_.y() * 0.5 - z_dir * size_.z() * 0.5;
    corners[2] = center_ - dir * size_.x() * 0.5 - ortho_dir * size_.y() * 0.5 - z_dir * size_.z() * 0.5;
    corners[3] = center_ + dir * size_.x() * 0.5 - ortho_dir * size_.y() * 0.5 - z_dir * size_.z() * 0.5;

    corners[4] = center_ + dir * size_.x() * 0.5 + ortho_dir * size_.y() * 0.5 + z_dir * size_.z() * 0.5;
    corners[5] = center_ - dir * size_.x() * 0.5 + ortho_dir * size_.y() * 0.5 + z_dir * size_.z() * 0.5;
    corners[6] = center_ - dir * size_.x() * 0.5 - ortho_dir * size_.y() * 0.5 + z_dir * size_.z() * 0.5;
    corners[7] = center_ + dir * size_.x() * 0.5 - ortho_dir * size_.y() * 0.5 + z_dir * size_.z() * 0.5;
    return corners;
}

void RotatedBox::fromCorners(const std::vector<Eigen::Vector3d> &corners) {
    RENSURE(corners.size() == 8);
    size_.x() = (corners[0] - corners[1]).norm();
    size_.y() = (corners[2] - corners[1]).norm();
    size_.z() = (corners[0] - corners[4]).norm();
    center_.setZero();
    for (int i = 0; i < 8; ++i) {
        center_ += corners[i];
    }
    center_ /= 8.;

    Eigen::Vector3d dir = corners[0] - corners[1];
    yaw_ = std::atan2(dir.y(), dir.x());
}

void RotatedBox::transform(const Eigen::Matrix4d &t_mat) {
    {   // center
        Eigen::Vector4d tt_pt;
        tt_pt << center_.x(), center_.y(), center_.z(), 1.;
        tt_pt = t_mat * tt_pt;
        center_.x() = tt_pt.x();
        center_.y() = tt_pt.y();
        center_.z() = tt_pt.z();
    }
    {  // yaw
        Eigen::Vector3d orientation(std::cos(yaw_),
                                    std::sin(yaw_), 0.);
        Eigen::Vector4d tt_pt;
        tt_pt << orientation.head(3), 0.f;
        tt_pt = t_mat * tt_pt;
        yaw_ = std::atan2(tt_pt.y(), tt_pt.x());
    }
}

}  // namespace rally
