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
#ifndef RALLY_CORE_CONTAINERS_SHORT_ARRAY_DETAILS_SHORT_ARRAY_2F_ARITHMETIC_H
#define RALLY_CORE_CONTAINERS_SHORT_ARRAY_DETAILS_SHORT_ARRAY_2F_ARITHMETIC_H

#include "rally/core/containers/details/short_array_arithmetic.h"

namespace rally {

inline ShortArray2f createUnitVec2d(const float rad) noexcept {
    return ShortArray2f(std::cos(rad), std::sin(rad));
}

inline float getRad(const ShortArray2f &vec) noexcept {
    return std::atan2(vec.y, vec.x);
}

inline float distance(const ShortArray2f &lhs, const ShortArray2f &rhs) noexcept {
    return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

inline float crossProd(const ShortArray2f &lhs, const ShortArray2f &rhs) noexcept {
    return lhs.x * rhs.y - lhs.y * rhs.x;
}

inline float crossProd(const ShortArray2f &start_point, const ShortArray2f &end_point_1,
                       const ShortArray2f &end_point_2) {
    return crossProd(end_point_1 - start_point, end_point_2 - start_point);
}

inline float innerProd(const ShortArray2f &lhs, const ShortArray2f &rhs) noexcept {
    return lhs.x * rhs.x + lhs.y * rhs.y;
}

inline float innerProd(const ShortArray2f &start_point, const ShortArray2f &end_point_1,
                       const ShortArray2f &end_point_2) {
    return innerProd(end_point_1 - start_point, end_point_2 - start_point);
}

inline ShortArray2f rotateVec(const ShortArray2f &in_vec, const float rad) noexcept {
    return ShortArray2f(in_vec.x * std::cos(rad) - in_vec.y * std::sin(rad),
                        in_vec.x * std::sin(rad) + in_vec.y * std::cos(rad));
}

}  // namespace rally

#endif  // RALLY_CORE_CONTAINERS_SHORT_ARRAY_DETAILS_SHORT_ARRAY_2F_ARITHMETIC_H
