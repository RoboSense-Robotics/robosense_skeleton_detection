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
#ifndef RALLY_COMMON_BASIC_TYPE_OBJECT_H
#define RALLY_COMMON_BASIC_TYPE_OBJECT_H

#include <Eigen/Dense>
#include "rally/utils/utils.h"
#include "rally/common/basic_type/rotated_box.h"
#include "rally/common/basic_type/object_type.h"

namespace rally {

struct Object3D {
    double timestamp;
    ObjectType type;
    Eigen::Vector3d velocity;
    Eigen::Vector3d accelerate;
    double yaw_rate;
    RotatedBox box;
    int64_t track_id;

    ///@brief 对Object3D的坐标变换
    void transform(const Eigen::Matrix4d &t_mat);
};

inline std::ostream &operator<<(std::ostream &os, const Object3D &p) {
    os << "Object3D: " << std::endl;
    os << "{timestamp: " << std::to_string(p.timestamp) << std::endl <<
    ", velocity: " << p.velocity.transpose() << std::endl <<
    ", accelerate: " << p.accelerate.transpose() << std::endl <<
    ", yaw_rate: " << std::to_string(p.yaw_rate) << std::endl <<
    ", box: " << p.box << std::endl <<
    ", track_id " << std::to_string(p.track_id) << "}";
    return os;
}

}  // namespace rally

#endif  // RALLY_COMMON_BASIC_TYPE_OBJECT_H
