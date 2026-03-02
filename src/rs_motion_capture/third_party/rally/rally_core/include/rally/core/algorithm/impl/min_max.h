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
#ifndef RALLY_CORE_ALGORITHM_IMPL_MIN_MAX_H
#define RALLY_CORE_ALGORITHM_IMPL_MIN_MAX_H

namespace rally {

template<typename T>
inline constexpr T max(const T &left) noexcept {
    return left;
}

template<typename T>
inline constexpr T max(const T &left, const T &right) noexcept {
    return (right < left) ? left : right;
}

template<typename T, typename... Targs>
inline constexpr T max(const T &left, const T &right, const Targs &... args) noexcept {
    return max(max(left, right), args...);
}

template<typename T>
inline constexpr T min(const T &left) noexcept {
    return left;
}

template<typename T>
inline constexpr T min(const T &left, const T &right) noexcept {
    return (left < right) ? left : right;
}

template<typename T, typename... Targs>
inline constexpr T min(const T &left, const T &right, const Targs &... args) noexcept {
    return min(min(left, right), args...);
}

}  // namespace rally

#endif  // RALLY_CORE_ALGORITHM_IMPL_MIN_MAX_H
