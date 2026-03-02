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
#ifndef RALLY_CORE_ALGORITHM_GEOMETRY_KALMAN_FILTER_H
#define RALLY_CORE_ALGORITHM_GEOMETRY_KALMAN_FILTER_H

#include <eigen3/Eigen/Dense>

namespace rally {
/// @brief Implements a discrete-time Kalman filter.
/// @tparam T
/// @tparam XN dimension of state
/// @tparam ZN dimension of measurement
/// @tparam UN dimension of controls
template<typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
class KalmanFilter {
    RALLY_STATIC_ASSERT(std::is_floating_point<T>::value);
public:
    /// @brief Constructor which defers initialization until the initial state
    /// distribution parameters are set (with SetStateEstimate), typically on the first measurement
    KalmanFilter();

    virtual ~KalmanFilter() {}

    /// @brief Sets the initial state belief distribution.
    /// @param x Mean of the state belief distribution
    /// @param P Covariance of the state belief distribution
    void setStateEstimate(const Eigen::Matrix<T, XN, 1> &x,
                          const Eigen::Matrix<T, XN, XN> &P);

    /// @brief Constructor which fully initializes the Kalman filter
    /// @param x Mean of the state belief distribution
    /// @param P Covariance of the state belief distribution
    KalmanFilter(const Eigen::Matrix<T, XN, 1> &x,
                 const Eigen::Matrix<T, XN, XN> &P);

    /// @brief the system transition function under zero control.
    /// @param F New transition matrix
    void setTransitionMatrix(const Eigen::Matrix<T, XN, XN> &F) { F_ = F; }

    /// @brief Changes the covariance matrix of the transition noise.
    /// @param Q New covariance matrix
    void setProcessNoise(const Eigen::Matrix<T, XN, XN> &Q) { Q_ = Q; }

    /// @brief the measurement matrix, which maps states to measurement.
    /// @param H New measurement matrix
    void setMeasurementMatrix(const Eigen::Matrix<T, ZN, XN> &H) { H_ = H; }

    /// @brief the covariance matrix of the measurement noise.
    /// @param R New covariance matrix
    void setMeasurementNoise(const Eigen::Matrix<T, ZN, ZN> &R) { R_ = R; }

    /// @brief the covariance matrix of current state belief distribution.
    /// @param P New state covariance matrix
    void setStateCovariance(const Eigen::Matrix<T, XN, XN> &P) { P_ = P; }

    /// @brief the control matrix in the state transition rule.
    /// @param B New control matrix
    void setControlMatrix(const Eigen::Matrix<T, XN, UN> &B) { B_ = B; }

    /// @brief Get the system transition function under zero control.
    /// @return Transition matrix.
    const Eigen::Matrix<T, XN, XN> &getTransitionMatrix() const { return F_; }

    /// @brief Get the covariance matrix of the transition noise.
    /// @return Covariance matrix
    const Eigen::Matrix<T, XN, XN> &getProcessNoise() const { return Q_; }

    /// @brief Get the measurement matrix, which maps states to measurement.
    /// @return Measurement matrix
    const Eigen::Matrix<T, ZN, XN> &getMeasurementMatrix() const { return H_; }

    /// @brief Get the covariance matrix of the measurement noise.
    /// @return Covariance matrix
    const Eigen::Matrix<T, ZN, ZN> &getMeasurementNoise() const { return R_; }

    /// @brief Get the control matrix in the state transition rule.
    /// @return Control matrix
    const Eigen::Matrix<T, XN, UN> &getControlMatrix() const { return B_; }

    /// @brief Updates the state belief distribution given the control input u.
    /// @param u param u Control input (by default, zero)
    void predict(const Eigen::Matrix<T, UN, 1> &u = Eigen::Matrix<T, UN, 1>::Zero());

    /// @brief Updates the state belief distribution given an measurement z.
    /// @param z measurement
    void update(const Eigen::Matrix<T, ZN, 1> &z);

    /// @brief Gets mean of our current state belief distribution
    /// @return State vector
    Eigen::Matrix<T, XN, 1> getStateEstimate() const { return x_; }

    /// @brief Gets covariance of our current state belief distribution
    /// @return Covariance matrix
    Eigen::Matrix<T, XN, XN> getStateCovariance() const { return P_; }

    /// @brief Gets debug string containing detailed information about the filter.
    /// @return
    std::string infos() const;

    /// @brief Get initialization state of the filter
    /// @return return True if the filter is initialized
    bool isInitialized() const { return is_initialized_; }

private:
    const std::string name() const {
        return "KalmanFilter";
    }

    /// @brief Mean of current state belief distribution
    Eigen::Matrix<T, XN, 1> x_;

    /// @brief Covariance of current state belief dist
    Eigen::Matrix<T, XN, XN> P_;

    /// @brief State transition matrix under zero control
    Eigen::Matrix<T, XN, XN> F_;

    /// @brief Covariance of the state transition noise
    Eigen::Matrix<T, XN, XN> Q_;

    /// @brief measurement matrix
    Eigen::Matrix<T, ZN, XN> H_;

    /// @brief Covariance of measurement noise
    Eigen::Matrix<T, ZN, ZN> R_;

    /// @brief Control matrix in state transition rule
    Eigen::Matrix<T, XN, UN> B_;

    /// @brief Innovation; marked as member to prevent memory re-allocation.
    Eigen::Matrix<T, ZN, 1> y_;

    /// @brief Innovation covariance; marked as member to prevent memory re-allocation.
    Eigen::Matrix<T, ZN, ZN> S_;

    /// @brief Kalman gain; marked as member to prevent memory re-allocation.
    Eigen::Matrix<T, XN, ZN> K_;

    /// @brief true if SetStateEstimate has been called.
    bool is_initialized_ = false;
};
}  // namespace rally

#include "rally/core/algorithm/geometry/impl/kalman_filter.h"

#endif  // RALLY_CORE_ALGORITHM_GEOMETRY_KALMAN_FILTER_H
