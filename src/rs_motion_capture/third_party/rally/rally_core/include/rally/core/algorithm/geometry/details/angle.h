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
#ifndef RALLY_CORE_ALGORITHM_GEOMETRY_DETAILS_ANGLE_H
#define RALLY_CORE_ALGORITHM_GEOMETRY_DETAILS_ANGLE_H

namespace rally {

/// @brief Sums two angles
/// @param lhs An Angle object
/// @param rhs An Angle object
/// @return Result of addition
template <typename T>
Angle<T> operator+(Angle<T> lhs, Angle<T> rhs) {
    lhs += rhs;
    return lhs;
}

/// @brief Subtracts two angles
/// @param lhs An Angle object
/// @param rhs An Angle object
/// @return Result of subtraction
template <typename T>
Angle<T> operator-(Angle<T> lhs, Angle<T> rhs) {
    lhs -= rhs;
    return lhs;
}

/// @brief Multiplies an Angle by a scalar
/// @param lhs An Angle object
/// @param rhs A scalar
/// @return Result of multiplication
template <typename T, typename Scalar>
Angle<T> operator*(Angle<T> lhs, Scalar rhs) {
    lhs *= rhs;
    return lhs;
}

/// @brief Multiplies an Angle by a scalar
/// @param lhs An Angle object
/// @param rhs A scalar
/// @return Result of multiplication
template <typename T, typename Scalar>
Angle<T> operator*(Scalar lhs, Angle<T> rhs) {
    rhs *= lhs;
    return rhs;
}

/// @brief Divides an Angle by a scalar
/// @param lhs An Angle object
/// @param rhs A scalar
/// @return Result of division
template <typename T, typename Scalar>
Angle<T> operator/(Angle<T> lhs, Scalar rhs) {
    lhs /= rhs;
    return lhs;
}

/// @brief Divides an Angle by a scalar
/// @param lhs An Angle object
/// @param rhs A scalar
/// @return Result of division
template <typename T>
double operator/(Angle<T> lhs, Angle<T> rhs) {
    return static_cast<double>(lhs.raw()) / rhs.raw();
}

/// @brief Tests two Angle objects for equality
/// @param lhs An Angle object
/// @param rhs An Angle object
/// @return lhs == rhs
template <typename T>
bool operator==(Angle<T> lhs, Angle<T> rhs) {
    return lhs.raw() == rhs.raw();
}

/// @brief Tests two Angle objects for inequality
/// @param lhs An Angle object
/// @param rhs An Angle object
/// @return lhs != rhs
template <typename T>
bool operator!=(Angle<T> lhs, Angle<T> rhs) {
    return !(lhs == rhs);
}

}  // namespace rally

#endif  // RALLY_CORE_ALGORITHM_GEOMETRY_DETAILS_ANGLE_H
