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
#ifndef RALLY_UTILS_DETAILS_EQUAL_H
#define RALLY_UTILS_DETAILS_EQUAL_H

#include <cmath>
#include <limits>
#include <algorithm>
#include <type_traits>

namespace rally {

/// @brief Integral type equal
template<typename T>
inline typename std::enable_if<std::is_integral<T>::value, bool>::type
isEqual(const T &lhs, const T &rhs) {
    return lhs == rhs;
}

/// @brief Floating point type equal
template<typename T>
inline typename std::enable_if<std::is_floating_point<T>::value, bool>::type
isEqual(const T &lhs, const T &rhs) {
    return std::abs(lhs - rhs) < 1.e-6;
}

}  // namespace rally

#endif  // RALLY_UTILS_DETAILS_EQUAL_H
