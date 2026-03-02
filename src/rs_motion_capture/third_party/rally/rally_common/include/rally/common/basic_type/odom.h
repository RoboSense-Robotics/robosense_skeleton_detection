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
#ifndef RALLY_COMMON_BASIC_TYPE_ODOM_H
#define RALLY_COMMON_BASIC_TYPE_ODOM_H

#include <Eigen/Dense>
#include "rally/utils/utils.h"

namespace rally {

struct Odom {
    using Ptr = std::shared_ptr<Odom>;

    double timestamp{0.};
    double pose_x{0.};
    double pose_y{0.};
    double pose_z{0.};

    double orientation_x{0.};
    double orientation_y{0.};
    double orientation_z{0.};
    double orientation_w{1.};

    Eigen::Affine3d toAffine3d() const;

    // this function need test
    void fromAffine3d(const Eigen::Affine3d &t);
};

inline std::ostream &operator<<(std::ostream &os, const Odom &p) {
    os << "{Odom: " << "timestamp: " << std::to_string(p.timestamp)
       << " s. pose_x: " << std::to_string(p.pose_x)
       << ", pose_y: " << std::to_string(p.pose_y)
       << ", pose_z: " << std::to_string(p.pose_z)
       << ", orientation_x: " << std::to_string(p.orientation_x)
       << ", orientation_y: " << std::to_string(p.orientation_y)
       << ", orientation_z: " << std::to_string(p.orientation_z)
       << ", orientation_w: " << std::to_string(p.orientation_w) << "}";
    return os;
}

struct Pose {
    double timestamp;
    double x{0.};
    double y{0.};
    double z{0.};
    double roll{0.};
    double pitch{0.};
    double yaw{0.};

    Eigen::Matrix4d toMat() const;

    void fromMat(const Eigen::Matrix4d &t);
};

inline std::ostream &operator<<(std::ostream &os, const Pose &p) {
    os << "{Pose: " << "timestamp: " << std::to_string(p.timestamp)
       << " s. x: " << std::to_string(p.x)
       << ", y: " << std::to_string(p.y)
       << ", z: " << std::to_string(p.z)
       << ", roll: " << std::to_string(p.roll)
       << ", pitch: " << std::to_string(p.pitch)
       << ", yaw: " << std::to_string(p.yaw) << "}";
    return os;
}

}  // namespace rally

#endif  // RALLY_COMMON_BASIC_TYPE_ODOM_H
