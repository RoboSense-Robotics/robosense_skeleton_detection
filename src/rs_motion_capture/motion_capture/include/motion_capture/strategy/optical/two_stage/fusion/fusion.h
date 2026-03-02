//
// Created by sti on 2025/6/11.
//

#ifndef MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_FUSION_FUSION_H
#define MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_FUSION_FUSION_H

#include <ceres/ceres.h>
#include "motion_capture/common/message.h"
#include <rs_log/init.h>

namespace robosense {
namespace motion_capture {

struct SmoothnessResidual {
  SmoothnessResidual(Point3f prev, float beta) : prev_(prev), beta_(beta) {}

  template <typename T>
  bool operator()(const T* const pi, T* residuals) const {
    residuals[0] = (pi[0] - T(prev_.x)) * T(beta_);
    residuals[1] = (pi[1] - T(prev_.y)) * T(beta_);
    residuals[2] = (pi[2] - T(prev_.z)) * T(beta_);
    return true;
  }

private:
  Point3f prev_;
  float beta_;
};

struct ProjectionResidual {
  ProjectionResidual(const Point2f &observed, const cv::Matx33f &K,
                     const Eigen::Matrix3f &R, const Eigen::Vector3f &T,
                     float weight)
          : observed_(observed), fx_(K(0,0)), fy_(K(1,1)), cx_(K(0,2)), cy_(K(1,2)),
            R_(R.transpose()), T_(T), weight_(weight) // 注意：预先转置用于逆变换
  {}

  template <typename T>
  bool operator()(const T* const pt_world, T* residuals) const {
    Eigen::Matrix<T, 3, 1> X_w(pt_world[0], pt_world[1], pt_world[2]);

    // 世界转相机：X_c = R^T * (X_w - T)
    Eigen::Matrix<T, 3, 1> T_eig = T_.cast<T>();
    Eigen::Matrix<T, 3, 1> X_c = R_.template cast<T>() * (X_w - T_eig);

    const T& X = X_c(0);
    const T& Y = X_c(1);
    const T& Z = X_c(2);

    T fx = T(fx_), fy = T(fy_), cx = T(cx_), cy = T(cy_);
    T w = T(weight_);

    T u = fx * X / Z + cx;
    T v = fy * Y / Z + cy;

    residuals[0] = w * (u - T(observed_.x)) / T(100.0);
    residuals[1] = w * (v - T(observed_.y)) / T(100.0);
    return true;
  }

private:
  const Point2f observed_;
  const float fx_, fy_, cx_, cy_;
  const Eigen::Matrix3f R_;  // 已转置的 R^T
  const Eigen::Vector3f T_;  // 原始平移 T
  const float weight_;
};

struct BoneLengthResidual {
  BoneLengthResidual(float target_length, float beta)
          : target_length_(target_length), beta_(beta) {}

  template <typename T>
  bool operator()(const T* const pi, const T* const pj, T* residuals) const {
    T dx = pi[0] - pj[0];
    T dy = pi[1] - pj[1];
    T dz = pi[2] - pj[2];
    T dist = ceres::sqrt(dx * dx + dy * dy + dz * dz);
    residuals[0] = (dist - T(target_length_)) * T(beta_);
    return true;
  }

private:
  float target_length_;
  float beta_;
};

class PoseFusionOptimization {
public:
  using Ptr = std::shared_ptr<PoseFusionOptimization>;

  PoseFusionOptimization() = default;

  void init(const YAML::Node& cfg_node);

  void process(const Msg::Ptr &msg_ptr);

  std::string name() const { return "PoseFusionOptimization"; }

private:
  std::vector<Point3f> getFusionPoseInit(const Msg::Ptr &msg_ptr);

  std::vector<Point3f> ceresSolverOptimization(const Msg::Ptr &msg_ptr,
                                               const std::vector<Point3f> &init_pose);

  std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, std::pair<Eigen::Vector3f, Eigen::Quaternionf>>
  calculateEndEffectorPose(const Msg::Ptr &msg_ptr, const std::vector<Point3f> &pose);

  Eigen::Quaternionf computeHandRotationFromBone(
          const Eigen::Vector3f& p1,
          const Eigen::Vector3f& p2,
          const Eigen::Vector3f& p3);

private:
  // fusion
  std::vector<int> opt_idxs_ = {0, 1, 2, 3, 4, 5, 6, 9, 10, 11, 12, 13, 14};
  bool use_opt_ = true;
  float gamma_proj_ = 0.3f;
  float length_beta_ = 1.0f;
  float smooth_beta_ = 0.3f;
  float obs3d_beta_ = 0.3f;

  std::vector<Point3f> pose3d_fusion_history_;

  std::map<size_t, std::vector<std::pair<size_t, size_t>>> node2bone_ = {
          {0, {{0, 1}, {0, 4}}},
          {1, {{0, 1}, {1, 2}}},
          {2, {{1, 2}, {2, 3}}},
          {3, {{2, 3}, {3, 7}}},
          {7, {{3, 7}}},
          {4, {{0, 4}, {4, 5}}},
          {5, {{4, 5}, {5, 6}}},
          {6, {{5, 6}, {6, 8}}},
          {8, {{6, 8}}},
          {9, {{0, 9}, {9, 10}}},
          {10, {{9, 10}, {10, 11}}},
          {11, {{10, 11}}},
          {12, {{0, 12}, {12, 13}}},
          {13, {{12, 13}, {13, 14}}},
          {14, {{13, 14}}},
  };
  std::map<size_t, std::vector<size_t>> node2line_ = {
          {0, {0, 3}},
          {1, {0, 1}},
          {2, {1, 2}},
          {3, {2, 6}},
          {7, {6}},
          {4, {3, 4}},
          {5, {4, 5}},
          {6, {5, 7}},
          {8, {7}},
          {9, {8, 9}},
          {10, {9, 10}},
          {11, {10}},
          {12, {11, 12}},
          {13, {12, 13}},
          {14, {13}},
  };
  std::vector<std::pair<int, int>> opt_line_set_ = {
          {0, 1},
          {1, 2},
          {2, 3},
          {0, 4},
          {4, 5},
          {5, 6},
          {3, 7},
          {6, 8},
          {0, 9},
          {9, 10},
          {10, 11},
          {0, 12},
          {12, 13},
          {13, 14}
  };
  std::map<rally::CameraEnum, std::vector<float>> bone_len_map_ = {
          {rally::CameraEnum::left_ac_camera, { 0.31f, 0.28f, 0.22f, 0.31f, 0.28f, 0.22f, 0.08f, 0.08f }},
          {rally::CameraEnum::right_ac_camera, { 0.31f, 0.28f, 0.22f, 0.31f, 0.28f, 0.22f, 0.08f, 0.08f }}
  };

  std::vector<float> bone_fusion_len_;

  std::vector<rally::CameraEnum> camera_list_ = { rally::CameraEnum::left_ac_camera,
                                                 rally::CameraEnum::right_ac_camera};

  struct OptimizationLoss {
    float total_cost = 0.0;
    float bone_length_cost = 0.0;
    float projection_cost = 0.0;
    float smoothness_cost = 0.0;
    float obs3d_cost = 0.0;

    void reset() {
      total_cost = 0.0;
      bone_length_cost = 0.0;
      projection_cost = 0.0;
      smoothness_cost = 0.0;
      obs3d_cost = 0.0;
    }

    void sum() {
      total_cost = bone_length_cost + projection_cost + smoothness_cost + obs3d_cost;
    }
  } loss_;

  std::string glove_type_ = "noiton"; // 手套类型
  float world_x_bias_ = 0.0;
  float world_y_bias_ = 0.0;
  float world_z_bias_ = 0.0;

  TimeRecorder::Ptr time_recorder_ptr_;

};

}
}

#endif //MOTION_CAPTURE_STRATEGY_OPTICAL_TWO_STAGE_FUSION_FUSION_H
