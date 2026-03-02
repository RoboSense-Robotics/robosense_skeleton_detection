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
#ifndef RALLY_COMMON_AXIS_TYPE_H
#define RALLY_COMMON_AXIS_TYPE_H

#include <map>
#include <string>

namespace rally {

enum class AxisType : uint8_t {
    LIDAR_AIXS = 0,
    VEHICLE_AXIS = 1,
    GLOBAL_AIXS = 2,
};

const std::map<AxisType, std::string> kAxisType2NameMap = {
{AxisType::LIDAR_AIXS,   "LIDAR_AIXS"},
{AxisType::VEHICLE_AXIS, "VEHICLE_AXIS"},
{AxisType::GLOBAL_AIXS,  "GLOBAL_AIXS"},
};

const std::map<std::string, AxisType> kAxisTypeName2TypeMap = {
{"LIDAR_AIXS",   AxisType::LIDAR_AIXS},
{"VEHICLE_AXIS", AxisType::VEHICLE_AXIS},
{"GLOBAL_AIXS",  AxisType::GLOBAL_AIXS},
};

inline std::ostream &operator<<(std::ostream &os, const AxisType &p) {
    os << "AxisType: " << kAxisType2NameMap.at(p) << std::endl;
    return os;
}

}  // namespace rally

#endif  // RALLY_COMMON_AXIS_TYPE_H
