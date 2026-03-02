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
#ifndef RALLY_COMMON_MSG_LOC_ODOM_H
#define RALLY_COMMON_MSG_LOC_ODOM_H

#include <Eigen/Dense>
#include "rally/common/msg/header.h"
#include "rally/common/basic_type/basic_type.h"

namespace rally {

struct LocOdom {
    using Ptr = std::shared_ptr<LocOdom>;

    Header header;

    Odom odom;
    Eigen::Vector3d vel; // velocity (body frame) at the Vehicle origin, [vx,vy,vz]
    Eigen::Vector3d acc; // acceleration (body frame) at the Vehicle origin, [ax,ay,az]
    Eigen::Vector3d gyr; // angular velocity (body frame) at the Vehicle origin, [gx,gy,gz]
};

inline std::ostream &operator<<(std::ostream &os, const LocOdom &p) {
    os << "LocOdom: " << std::endl;
    os << "{header: " << p.header <<
       ", Odom: " <<p.odom <<
       ", acc: " <<p.acc.transpose() <<
       ", gyr: " <<p.gyr.transpose() <<
       ", vel " << p.vel.transpose() << std::endl;
    os << "}";
    return os;
}

}  // namespace rally

#endif  // RALLY_COMMON_MSG_LOC_ODOM_H
