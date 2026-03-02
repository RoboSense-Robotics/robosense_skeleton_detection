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
#ifndef RALLY_CORE_ALGORITHM_MIN_MAX_H
#define RALLY_CORE_ALGORITHM_MIN_MAX_H

namespace rally {

/// @brief Returns the maximum gained with operator<() of an arbitrary amount
///        of variables of the same type. Helper function which is required as generic
///        recursive template endpoint.
/// @param T type which implements operator<()
/// @param[in] left value which should be compared
/// @return returns the given argument left
template<typename T>
constexpr T max(const T &left) noexcept;

/// @brief Returns the maximum gained with operator<() of an arbitrary amount
///        of variables of the same type. Helper function which takes two arguments and returns the
///        greater one.
/// @param T type which implements operator<()
/// @param[in] left value which should be compared
/// @param[in] right value which should be compared
/// @return returns the maximum value of the set {left, right}
template<typename T>
constexpr T max(const T &left, const T &right) noexcept;

/// @brief Returns the maximum gained with operator<() of an arbitrary amount
///        of variables of the same type.
/// @param T type which implements operator<()
/// @param[in] left value which should be compared
/// @param[in] right value which should be compared
/// @param[in] args... an arbitrary amount of values
/// @return returns the maximum value of the set {left, right, args...}
template<typename T, typename... Targs>
constexpr T max(const T &left, const T &right, const Targs &... args) noexcept;

/// @brief Returns the minimum gained with operator<() of an arbitrary amount
///        of variables of the same type. Helper function which is required as generic
///        recursive template endpoint.
/// @param T type which implements operator<()
/// @param[in] left value which should be compared
/// @return returns the given argument left
template<typename T>
constexpr T min(const T &left) noexcept;

/// @brief Returns the minimum gained with operator<() of an arbitrary amount
///        of variables of the same type. Helper function which takes two arguments and returns the
///        smaller one.
/// @param T type which implements operator<()
/// @param[in] left value which should be compared
/// @param[in] right value which should be compared
/// @return returns the minimum of the set {left, right}
template<typename T>
constexpr T min(const T &left, const T &right) noexcept;

/// @brief Returns the minimum gained with operator<() of an arbitrary amount
///        of variables of the same type.
/// @param T type which implements operator<()
/// @param[in] left value which should be compared
/// @param[in] right value which should be compared
/// @param[in] args... an arbitrary amount of values
/// @return returns the minimum of the set {left, right, args...}
template<typename T, typename... Targs>
constexpr T min(const T &left, const T &right, const Targs &... args) noexcept;

}  // namespace rally

#include "rally/core/algorithm/impl/min_max.h"

#endif  // RALLY_CORE_ALGORITHM_MIN_MAX_H
