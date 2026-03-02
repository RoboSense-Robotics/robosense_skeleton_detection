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
#ifndef RALLY_CORE_ALGORITHM_GEOMETRY_IMPL_KALMAN_FILTER_H
#define RALLY_CORE_ALGORITHM_GEOMETRY_IMPL_KALMAN_FILTER_H

namespace rally {

template<typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
KalmanFilter<T, XN, ZN, UN>::KalmanFilter() {
    F_.setIdentity();
    Q_.setZero();
    H_.setIdentity();
    R_.setZero();
    B_.setZero();

    x_.setZero();
    P_.setZero();
    y_.setZero();
    S_.setZero();
    K_.setZero();
}

template<typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
KalmanFilter<T, XN, ZN, UN>::KalmanFilter(const Eigen::Matrix<T, XN, 1> &x,
                                          const Eigen::Matrix<T, XN, XN> &P)
: KalmanFilter() {
    setStateEstimate(x, P);
}

template<typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
void KalmanFilter<T, XN, ZN, UN>::setStateEstimate(const Eigen::Matrix<T, XN, 1> &x,
                                                   const Eigen::Matrix<T, XN, XN> &P) {
    x_ = x;
    P_ = P;
    is_initialized_ = true;
}

template<typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
inline void KalmanFilter<T, XN, ZN, UN>::predict(const Eigen::Matrix<T, UN, 1> &u) {
    if (!is_initialized_) {
        RDEBUG << name() << ": not initialized!";
    }

    x_ = F_ * x_ + B_ * u;

    P_ = F_ * P_ * F_.transpose() + Q_;
}

template<typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
inline void KalmanFilter<T, XN, ZN, UN>::update(const Eigen::Matrix<T, ZN, 1> &z) {
    if (!is_initialized_) {
        RDEBUG << name() << ": not initialized!";
    }
    y_ = z - H_ * x_;

    S_ = static_cast<Eigen::Matrix<T, ZN, ZN>>(H_ * P_ * H_.transpose() + R_);

    Eigen::JacobiSVD<Eigen::Matrix<T, ZN, ZN>> svd(
            S_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<T, ZN, ZN> S_inverse_ = static_cast<Eigen::Matrix<T, ZN, ZN>>(
            svd.matrixV() *
            (svd.singularValues().array().abs() > 1.0e-6)
            .select(svd.singularValues().array().inverse(), 0)
            .matrix()
            .asDiagonal() *
            svd.matrixU().adjoint());

    K_ = static_cast<Eigen::Matrix<T, XN, ZN>>(P_ * H_.transpose() * S_inverse_);

    x_ = x_ + K_ * y_;

    P_ = static_cast<Eigen::Matrix<T, XN, XN>>(
    (Eigen::Matrix<T, XN, XN>::Identity() - K_ * H_) * P_);
}

template<typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
inline std::string KalmanFilter<T, XN, ZN, UN>::infos() const {
    Eigen::IOFormat clean_fmt(4, 0, ", ", " ", "[", "]");
    std::stringstream ss;
    ss << "F = " << F_.format(clean_fmt) << "\n"
       << "B = " << B_.format(clean_fmt) << "\n"
       << "H = " << H_.format(clean_fmt) << "\n"
       << "Q = " << Q_.format(clean_fmt) << "\n"
       << "R = " << R_.format(clean_fmt) << "\n"
       << "x = " << x_.format(clean_fmt) << "\n"
       << "P = " << P_.format(clean_fmt) << "\n";
    return ss.str();
}

}  // namespace rally

#endif  // RALLY_CORE_ALGORITHM_GEOMETRY_IMPL_KALMAN_FILTER_H
