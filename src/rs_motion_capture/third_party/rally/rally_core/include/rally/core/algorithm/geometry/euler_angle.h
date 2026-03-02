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
#ifndef RALLY_CORE_ALGORITHM_GEOMETRY_EULER_ANGLE_H
#define RALLY_CORE_ALGORITHM_GEOMETRY_EULER_ANGLE_H

#include "rally/utils/utils.h"
#include "rally/core/algorithm/geometry/geo_utils.h"
#include <eigen3/Eigen/Dense>

namespace rally {

/// @class EulerAngle
/// Any orientation of a rigid body on a 3-D space can be achieved by
/// composing three rotations about the axes of an orthogonal coordinate system.
/// These rotations are said to be extrinsic if the axes are assumed to be
/// motionless, and intrinsic otherwise. Here, we use an intrinsic referential,
/// which is relative to the car's orientation.
/// Our vehicle reference frame follows NovAtel's convention:
/// Right/Forward/Up (RFU) respectively for the axes x/y/z.
/// In particular, we describe the orientation of the car by three angles:
/// 1) the pitch, in (-pi/2, pi/2), corresponds to a rotation around the x-axis;
/// 2) the roll, in [-pi, pi), corresponds to a rotation around the y-axis;
/// 3) the yaw, in [-pi, pi), corresponds to a rotation around the z-axis.
/// The pitch is zero when the car is level and positive when the nose is up.
/// The roll is zero when the car is level and positive when the left part is up.
/// The yaw is zero when the car is facing North, and positive when facing West.
/// In turn, in the world frame, the x/y/z axes point to East/North/Up (ENU).
/// These angles represent the rotation from the world to the vehicle frames.
///
/// @brief Implements a class of Euler angles (actually, Tait-Bryan angles),
/// with intrinsic sequence ZXY.
///
/// @param T Number type: double or float
template<typename T>
class EulerAngle {
    RALLY_STATIC_ASSERT(std::is_floating_point<T>::value);
public:
    /// @brief Constructs an identity rotation.
    EulerAngle() : roll_(0), pitch_(0), yaw_(0) {}

    /// @brief Constructs a rotation using only yaw (i.e., around the z-axis).
    /// @param  yaw The yaw of the car
    explicit EulerAngle(T yaw) : roll_(0), pitch_(0), yaw_(yaw) {}

    /// @brief Constructs a rotation using arbitrary roll, pitch, and yaw.
    /// @param roll The roll of the car
    /// @param pitch The pitch of the car
    /// @param yaw The yaw of the car
    EulerAngle(T roll, T pitch, T yaw)
    : roll_(roll), pitch_(pitch), yaw_(yaw) {}

    /// @brief Constructs a rotation using components of a quaternion.
    /// @param qw Quaternion w-coordinate
    /// @param qx Quaternion x-coordinate
    /// @param qy Quaternion y-coordinate
    /// @param qz Quaternion z-coordinate
    EulerAngle(T qw, T qx, T qy, T qz)
    : roll_(std::atan2(static_cast<T>(2.0) * (qw * qy - qx * qz),
                       static_cast<T>(2.0) * (qw * qw + qz * qz) - static_cast<T>(1.0))),
      pitch_(std::asin(static_cast<T>(2.0) * (qw * qx + qy * qz))),
      yaw_(std::atan2(static_cast<T>(2.0) * (qw * qz - qx * qy),
                      static_cast<T>(2.0) * (qw * qw + qy * qy) - static_cast<T>(1.0))) {}

    /// @brief Constructs a rotation from quaternion.
    /// @param q Quaternion
    explicit EulerAngle(const Eigen::Quaternion<T> &q)
    : EulerAngle(q.w(), q.x(), q.y(), q.z()) {}

    /// @brief Getter for roll_
    /// @return The roll of the car
    T roll() const { return roll_; }

    /// @brief Getter for pitch_
    /// @return The pitch of the car
    T pitch() const { return pitch_; }

    /// @brief Getter for yaw_
    /// @return The yaw of the car
    T yaw() const { return yaw_; }

    /// @brief Normalizes roll_, pitch_, and yaw_ to [-PI, PI).
    void normalize() {
        roll_ = normalizeAngle(roll_);
        pitch_ = normalizeAngle(pitch_);
        yaw_ = normalizeAngle(yaw_);
    }

    /// @brief Verifies the validity of the specified rotation.
    /// @return True if -PI/2 < pitch < PI/2
    bool isValid() {
        normalize();
        return pitch_ < M_PI_2 && pitch_ > -M_PI_2;
    }

    /// @brief Converts to a quaternion with a non-negative scalar part
    /// @return Quaternion encoding this rotation.
    Eigen::Quaternion<T> toQuaternion() const {
        T coeff = static_cast<T>(0.5);
        T r = roll_ * coeff;
        T p = pitch_ * coeff;
        T y = yaw_ * coeff;

        T sr = std::sin(r);
        T sp = std::sin(p);
        T sy = std::sin(y);

        T cr = std::cos(r);
        T cp = std::cos(p);
        T cy = std::cos(y);

        T qw = cr * cp * cy - sr * sp * sy;
        T qx = cr * sp * cy - sr * cp * sy;
        T qy = cr * sp * sy + sr * cp * cy;
        T qz = cr * cp * sy + sr * sp * cy;
        if (qw < 0.0) {
            return {-qw, -qx, -qy, -qz};
        }
        return {qw, qx, qy, qz};
    }

private:
    T roll_;
    T pitch_;
    T yaw_;
};

using EulerAnglef = EulerAngle<float>;
using EulerAngled = EulerAngle<double>;

}  // namespace rally

#endif  // RALLY_CORE_ALGORITHM_GEOMETRY_EULER_ANGLE_H
