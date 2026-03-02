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
#ifndef RALLY_CORE_ALGORITHM_GEOMETRY_ANGLE_H
#define RALLY_CORE_ALGORITHM_GEOMETRY_ANGLE_H

#include <cmath>
#include <cstdint>
#include <limits>
#include "rally/utils/utils.h"

namespace rally {

/// @brief The Angle class uses an integer to represent an angle, and supports
/// commonly-used operations such as addition and subtraction,
/// as well as the use of trigonometric functions.
///
/// Having a specialized Angle class prevents code repetition, namely for tasks
/// such as computing angle differences or finding equivalent angles in some
/// specified interval, typically [-pi, pi).
/// Representing angles by integers has the following advantages:
/// 1) Finer level of precision control (< means "less precise than"):
/// Angle8 < Angle16 < float < Angle32 < double < Angle64.
/// 2) Angle8 and Angle16 allow super fast trigonometric functions
/// via a 64-KiB lookup table.
/// 3) Higher precision with the same representation size.
/// The use of the Angle class is encouraged.
/// In particular, Angle32 should be used for latitude/longitude (<1cm error).
/// Angle16 is precise enough for localization/object detection.
template <typename T>
class Angle {
    RALLY_STATIC_ASSERT(std::numeric_limits<T>::is_integer &&
                        std::numeric_limits<T>::is_signed);
public:

    /// @brief Constructs an Angle object from an angle in degrees (factory).
    /// @param value Angle in degrees
    /// @return Angle object
    static Angle from_deg(const double value) { return Angle(static_cast<T>(std::lround(value * DEG_TO_RAW))); }

    /// @brief Constructs an Angle object from an angle in radians (factory).
    /// @param value Angle in radians
    /// @return Angle object
    static Angle from_rad(const double value) { return Angle(static_cast<T>(std::lround(value * RAD_TO_RAW))); }

    /// @brief Constructs an Angle object from raw internal value.
    /// @param value Angle in degrees
    /// @return Angle object
    explicit Angle(const T value = 0) : value_(value) {}

    /// @brief Getter of value_.
    /// @return Internal unsigned integer representation of angle
    T raw() const { return value_; }

    /// @brief Converts the internal representation to degrees.
    /// @return angle in degrees
    double to_deg() const { return value_ * RAW_TO_DEG; }

    /// @brief Converts the internal representation to radians.
    /// @return angle in radians
    double to_rad() const { return value_ * RAW_TO_RAD; }

    /// @brief Sums another angle to the current one.
    /// @param other Another Angle object
    /// @return Result of sum
    Angle operator+=(Angle other) {
        value_ = static_cast<T>(value_ + other.value_);
        return *this;
    }

    /// @brief Subtracts another angle from the current one.
    /// @param other Another Angle object
    /// @return Result of subtraction
    Angle operator-=(Angle other) {
        value_ = static_cast<T>(value_ - other.value_);
        return *this;
    }

    /// @brief Multiplies angle by scalar
    /// @param s A scalar
    /// @return Result of multiplication
    template <typename Scalar>
    Angle operator*=(Scalar s) {
        value_ = static_cast<T>(std::lround(value_ * s));
        return *this;
    }

    /// @brief Divides angle by scalar
    /// @param s A scalar
    /// @return Result of division
    template <typename Scalar>
    Angle operator/=(Scalar s) {
        value_ = static_cast<T>(std::lround(value_ / s));
        return *this;
    }

    /// @brief Internal representation of pi
    static constexpr T RAW_PI = std::numeric_limits<T>::min();

    /// @brief Internal representation of pi/2
    static constexpr T RAW_PI_2 =
    static_cast<T>(-(std::numeric_limits<T>::min() >> 1));

    /// @brief Used for converting angle units
    static constexpr double DEG_TO_RAW = RAW_PI / -180.0;

    /// @brief Used for converting angle units
    static constexpr double RAD_TO_RAW = RAW_PI * -M_1_PI;

    /// @brief Used for converting angle units
    static constexpr double RAW_TO_DEG = -180.0 / RAW_PI;

    /// @brief Used for converting angle units
    static constexpr double RAW_TO_RAD = -M_PI / RAW_PI;

private:
    T value_;
};

using Angle8 = Angle<int8_t>;
using Angle16 = Angle<int16_t>;
using Angle32 = Angle<int32_t>;
using Angle64 = Angle<int64_t>;

// Fast trigonometric functions. Single precision is sufficient for Angle16 and
// Angle8.
float sin(Angle16 a);
float cos(Angle16 a);
float tan(Angle16 a);
float sin(Angle8 a);
float cos(Angle8 a);
float tan(Angle8 a);

}  // namespace rally

#include "rally/core/algorithm/geometry/details/angle.h"

#endif  // RALLY_CORE_ALGORITHM_GEOMETRY_ANGLE_H
