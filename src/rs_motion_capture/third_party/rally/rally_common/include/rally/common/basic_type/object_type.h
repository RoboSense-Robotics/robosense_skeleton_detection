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
#ifndef RALLY_COMMON_BASIC_TYPE_OBJECT_TYPE_H
#define RALLY_COMMON_BASIC_TYPE_OBJECT_TYPE_H

#include "rally/utils/utils.h"

namespace rally {

enum class ObjectType : uint8_t {
    CAR = 0,
    BUS = 1,
    TRUCK = 2,
    CONSTRN_VEH = 3,
    CYCLIST = 4,
    TRI_CYCLE = 5,
    PEDESTRIAN = 6,
    BARROW = 7,
    ANIMAL = 8,
    TRAFFIC_CONE = 9,
    WARN_TRIANGLE = 10,
    TRUCKHEAD = 11,
    BIRD = 12,
    ERROR = 255,
};

const std::map<ObjectType, std::string> kObjectType2NameMap = {
{ObjectType::CAR,           "CAR"},
{ObjectType::BUS,           "BUS"},
{ObjectType::TRUCK,         "TRUCK"},
{ObjectType::CONSTRN_VEH,   "CONSTRN_VEH"},
{ObjectType::CYCLIST,       "CYCLIST"},
{ObjectType::TRI_CYCLE,     "TRI_CYCLE"},
{ObjectType::PEDESTRIAN,   "PEDESTRIAN"},
{ObjectType::BARROW,        "BARROW"},
{ObjectType::ANIMAL,        "ANIMAL"},
{ObjectType::TRAFFIC_CONE,  "TRAFFIC_CONE"},
{ObjectType::WARN_TRIANGLE, "WARN_TRIANGLE"},
{ObjectType::TRUCKHEAD,     "TRUCKHEAD"},
{ObjectType::BIRD,     "BIRD"},
{ObjectType::ERROR,         "ERROR"},
};

const std::map<std::string, ObjectType> kObjectTypeName2TypeMap = {
{"CAR",           ObjectType::CAR},
{"BUS",           ObjectType::BUS},
{"TRUCK",         ObjectType::TRUCK},
{"CONSTRN_VEH",   ObjectType::CONSTRN_VEH},
{"CYCLIST",       ObjectType::CYCLIST},
{"TRI_CYCLE",     ObjectType::TRI_CYCLE},
{"PEDESTRIAN",   ObjectType::PEDESTRIAN},
{"BARROW",        ObjectType::BARROW},
{"ANIMAL",        ObjectType::ANIMAL},
{"TRAFFIC_CONE",  ObjectType::TRAFFIC_CONE},
{"WARN_TRIANGLE", ObjectType::WARN_TRIANGLE},
{"TRUCKHEAD",     ObjectType::TRUCKHEAD},
{"BIRD",     ObjectType::BIRD},
{"ERROR",         ObjectType::ERROR},
};

inline std::ostream &operator<<(std::ostream &os, const ObjectType &p) {
    os << "ObjectType: " << kObjectType2NameMap.at(p) << std::endl;
    return os;
}

}  // namespace rally

#endif  // RALLY_COMMON_BASIC_TYPE_OBJECT_TYPE_H
