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
#ifndef RALLY_COMMON_BASIC_TYPE_POINT_TYPE_H
#define RALLY_COMMON_BASIC_TYPE_POINT_TYPE_H

#include <pcl/point_types.h>

struct RsPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring = 0;
    double timestamp = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT, (float, x, x)(float, y, y)(float, z, z)
(float, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp))

using RsPoint = RsPointXYZIRT;

namespace rally {

inline std::ostream &operator<<(std::ostream &os, const RsPoint &p) {
    os << "{RsPoint: " << "x: " << std::to_string(p.x)
       << ", y: " << std::to_string(p.y)
       << ", z: " << std::to_string(p.z)
       << ", intensity: " << std::to_string(p.intensity)
       << ", ring: " << std::to_string(p.ring)
       << ", timestamp: " << std::to_string(p.timestamp) << " s."
       << "}" ;
    return os;
}

}  // namespace rally

#endif  // RALLY_COMMON_BASIC_TYPE_POINT_TYPE_H
