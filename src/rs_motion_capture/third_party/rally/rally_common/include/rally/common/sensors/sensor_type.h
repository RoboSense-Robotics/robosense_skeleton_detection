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
#ifndef RALLY_COMMON_SENSORS_SENSOR_TYPE_H
#define RALLY_COMMON_SENSORS_SENSOR_TYPE_H

#include <map>
#include <string>

namespace rally {

enum class SensorType : uint8_t {
    LIDAR = 0,
    CAMERA = 1,
    OTHERS = 2,
};

const std::map<SensorType, std::string> kSensorType2NameMap = {
{SensorType::LIDAR,  "Lidar"},
{SensorType::CAMERA, "Camera"},
{SensorType::OTHERS, "Others"},
};

const std::map<std::string, SensorType> kSensorTypeName2TypeMap = {
{"Lidar",  SensorType::LIDAR},
{"Camera", SensorType::CAMERA},
{"Others", SensorType::OTHERS},
};

inline std::ostream &operator<<(std::ostream &os, const SensorType &p) {
    os << "SensorType: " << kSensorType2NameMap.at(p) << std::endl;
    return os;
}

}  // namespace rally

#endif  // RALLY_COMMON_SENSORS_SENSOR_TYPE_H
