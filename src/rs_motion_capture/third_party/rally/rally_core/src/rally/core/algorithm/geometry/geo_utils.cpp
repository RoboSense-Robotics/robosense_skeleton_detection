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

#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>
#include "rally/core/algorithm/geometry/geo_utils.h"

namespace rally {

float normalizeAngle(const float rad) {
    float rad_res = rad;
    while (rad_res > rally::R_M_PI) {
        rad_res -= rally::R_M_DOUBLE_PI;
    }
    while (rad_res < -rally::R_M_PI) {
        rad_res += rally::R_M_DOUBLE_PI;
     }
    return rad_res;
}

float wrapAngle(const float rad) {
    auto res = normalizeAngle(rad - rally::R_M_PI);
    return res + rally::R_M_PI;
}

float degreeToRad(float degree) {
    float rad = degree / 180.f * rally::R_M_PI;
    return normalizeAngle(rad);
}

float radToDegree(float rad) {
    auto res_rad = normalizeAngle(rad);
    return res_rad / rally::R_M_PI * 180.f;
}

}  // namespace rally
