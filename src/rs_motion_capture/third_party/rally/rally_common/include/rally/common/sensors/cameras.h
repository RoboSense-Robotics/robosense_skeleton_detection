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
#ifndef RALLY_COMMON_SENSORS_CAMERAS_H
#define RALLY_COMMON_SENSORS_CAMERAS_H

#include "rally/utils/utils.h"

namespace rally {

enum class CameraEnum : uint8_t {
    left_ac_camera = 0,
    right_ac_camera = 1,
    fusion = 2,
};

const std::set<CameraEnum> kPinholeSet = {
CameraEnum::left_ac_camera,
CameraEnum::right_ac_camera,
};

const std::set<CameraEnum> kFisheyeSet = {
CameraEnum::fusion,
};

const std::map<CameraEnum, std::string> kCameraEnum2NameMap = {
{CameraEnum::left_ac_camera, "left_ac_camera"},
{CameraEnum::right_ac_camera, "right_ac_camera"},
{CameraEnum::fusion, "placeholder_camera"},
};

const std::map<std::string, CameraEnum> kCameraEnumName2TypeMap = {
{"left_ac_camera", CameraEnum::left_ac_camera},
{"right_ac_camera", CameraEnum::right_ac_camera},
{"placeholder_camera", CameraEnum::fusion},
};

inline std::ostream &operator<<(std::ostream &os, const CameraEnum &p) {
    os << "CameraEnum: " << kCameraEnum2NameMap.at(p) << std::endl;
    return os;
}

}  // namespace rally

#endif  // RALLY_COMMON_SENSORS_CAMERAS_H
