//
// Created by peter on 2025/6/19.
//

#ifndef MOTION_CAPTURE_ALGORITHM_GEOMETRY_KALMAN_H
#define MOTION_CAPTURE_ALGORITHM_GEOMETRY_KALMAN_H

#include <Eigen/Dense>

namespace robosense {
namespace motion_capture {

class KalmanFilter1D {
 public:
  KalmanFilter1D(float q = 1e-2f, float r = 1e-1f)
      : is_initialized_(false) {
    A_.setIdentity();
    H_ << 1, 0;

    Q_ = q * Eigen::Matrix2f::Identity();
    R_ = Eigen::Matrix<float, 1, 1>::Constant(r);
    P_ = Eigen::Matrix2f::Identity();
    x_.setZero();
  }

  float predict(double timestamp) {
    if (!is_initialized_) {
      // 第一次调用，初始化状态为观测值或 0
      x_ << 0.0f, 0.0f;
      last_timestamp_ = timestamp;
      is_initialized_ = true;
      return x_(0);
    }

    float dt = timestamp - last_timestamp_;
    last_timestamp_ = timestamp;

    A_ << 1, dt,
          0, 1;

    x_ = A_ * x_;
    P_ = A_ * P_ * A_.transpose() + Q_;
    return x_(0);
  }

  float update(float z) {
    if (!is_initialized_) {
      // 若还未初始化，则忽略 update
      return z;
    }

    float y = z - (H_ * x_)(0);
    float S = (H_ * P_ * H_.transpose())(0, 0) + R_(0, 0);
    Eigen::Vector2f K = P_ * H_.transpose() / S;

    x_ = x_ + K * y;
    P_ = (Eigen::Matrix2f::Identity() - K * H_) * P_;

    return x_(0);
  }

  float getPosition() const { return x_(0); }
  float getVelocity() const { return x_(1); }

 private:
  Eigen::Matrix2f A_;             // 状态转移矩阵
  Eigen::Matrix2f Q_;             // 过程噪声协方差
  Eigen::Matrix2f P_;             // 估计协方差
  Eigen::RowVector2f H_;          // 观测矩阵
  Eigen::Matrix<float, 1, 1> R_;  // 观测噪声协方差
  Eigen::Vector2f x_;             // 当前状态 [位置, 速度]

  double last_timestamp_ = 0.0f;
  bool is_initialized_;
};

class KalmanFilter2D {
public:
    KalmanFilter2D(float q = 1e-3f, float r = 1e-1f)
        : is_initialized(false), q_(q) {
        // 状态向量 x = [px, py, vx, vy]
        x_.setZero();

        // 状态转移矩阵 A
        A_.setIdentity();

        // 观测矩阵 H，只观测位置
        H_.setZero();
        H_.block<2,2>(0,0) = Eigen::Matrix2f::Identity();

        // 观测噪声 R
        R_ = r * Eigen::Matrix2f::Identity();

        // 初始协方差 P
        P_ = Eigen::Matrix<float,4,4>::Identity() * 1e3f; // 初始大不确定性
    }

    // 初始化
    void initialize(const Eigen::Vector2f& z, double timestamp) {
        x_.segment<2>(0) = z;       // 位置
        x_.segment<2>(2).setZero(); // 速度
        last_timestamp_ = timestamp;
        is_initialized = true;
    }

    // 预测
    Eigen::Vector2f predict(double timestamp) {
        if (!is_initialized) return Eigen::Vector2f::Zero();

        float dt = static_cast<float>(timestamp - last_timestamp_);
        last_timestamp_ = timestamp;

        // 状态转移矩阵 A
        A_.setIdentity();
        A_(0,2) = dt;
        A_(1,3) = dt;

        // 过程噪声 Q（基于速度噪声）
        float dt2 = dt * dt;
        Q_.setZero();
        Q_(0,0) = dt2/2;  Q_(0,2) = dt/2;
        Q_(1,1) = dt2/2;  Q_(1,3) = dt/2;
        Q_(2,0) = dt/2;   Q_(2,2) = 1;
        Q_(3,1) = dt/2;   Q_(3,3) = 1;
        Q_ *= q_;

        // 预测
        x_ = A_ * x_;
        P_ = A_ * P_ * A_.transpose() + Q_;

        return x_.segment<2>(0); // 返回预测位置
    }

    // 更新
    Eigen::Vector2f update(const Eigen::Vector2f& z) {
        if (!is_initialized) return z;

        Eigen::Vector2f y = z - H_ * x_; // 残差
        Eigen::Matrix2f S = H_ * P_ * H_.transpose() + R_;
        Eigen::Matrix<float,4,2> K = P_ * H_.transpose() * S.inverse();

        x_ = x_ + K * y;

        // Joseph form 更新协方差
        Eigen::Matrix<float,4,4> I = Eigen::Matrix<float,4,4>::Identity();
        P_ = (I - K * H_) * P_ * (I - K * H_).transpose() + K * R_ * K.transpose();

        return x_.segment<2>(0);
    }

    Eigen::Vector2f getPosition() const { return x_.segment<2>(0); }
    Eigen::Vector2f getVelocity() const { return x_.segment<2>(2); }
    bool is_initialized;

private:
    Eigen::Matrix<float,4,4> A_; // 状态转移
    Eigen::Matrix<float,4,4> P_; // 协方差
    Eigen::Matrix<float,4,4> Q_; // 过程噪声
    Eigen::Matrix<float,2,4> H_; // 观测矩阵
    Eigen::Matrix2f R_;          // 观测噪声
    Eigen::Matrix<float,4,1> x_; // 状态向量

    double last_timestamp_ = 0.0; // 时间戳用 double
    float q_; // 过程噪声系数
};



}  // namespace motion_capture
}  // namespace robosense

#endif  // MOTION_CAPTURE_ALGORITHM_GEOMETRY_KALMAN_H
