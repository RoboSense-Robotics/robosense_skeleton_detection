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
#ifndef RALLY_CORE_ALGORITHM_SAMPLE_CONSENSUS_DETAILS_SAC_MODEL_EIGEN_LINE_H
#define RALLY_CORE_ALGORITHM_SAMPLE_CONSENSUS_DETAILS_SAC_MODEL_EIGEN_LINE_H

#include <eigen3/Eigen/Dense>
#include "rally/utils/utils.h"
#include "rally/core/containers/containers.h"
#include "rally/core/algorithm/sample_consensus/details/sac_model.h"

namespace rally {

template<typename PointT>
class SampleConsensusModelEigenLine : public SampleConsensusModel<PointT> {
    using SampleConsensusModel<PointT>::model_name_;
    using SampleConsensusModel<PointT>::sample_size_;
    using SampleConsensusModel<PointT>::model_size_;
    using SampleConsensusModel<PointT>::input_;
    using SampleConsensusModel<PointT>::isModelValid;
    using SampleConsensusModel<PointT>::indices_;
    using SampleConsensusModel<PointT>::error_sqr_dists_;
public:
    using Ptr = std::unique_ptr<SampleConsensusModelEigenLine<PointT> >;

    /// @brief Constructor for base SampleConsensusModelLine.
    /// @param[in] cloud the input point cloud dataset
    /// @param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
    explicit SampleConsensusModelEigenLine(const std::vector<PointT> &cloud, bool random = false)
    : SampleConsensusModel<PointT>(cloud, random) {
        model_name_ = "SampleConsensusModelEigenLine";
        sample_size_ = 2;
        model_size_ = 6;
    }

    /// @brief Constructor for base SampleConsensusModelLine.
    /// @param[in] cloud the input point cloud dataset
    /// @param[in] indices a vector of point indices to be used from cloud
    /// @param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
    SampleConsensusModelEigenLine(const std::vector<PointT> &cloud,
                                  const std::vector<uint32_t> &indices,
                                  bool random = false)
    : SampleConsensusModel<PointT>(cloud, indices, random) {
        model_name_ = "SampleConsensusModelEigenLine";
        sample_size_ = 2;
        model_size_ = 6;
    }

    /// @brief Empty destructor
    ~SampleConsensusModelEigenLine() {}

    /// @brief Check whether the given index samples can form a valid line model, compute the model coefficients from
    /// these samples and store them internally in model_coefficients_. The line coefficients are represented by a
    /// point and a line direction
    /// @param[in] samples the point indices found as possible good candidates for creating a valid model
    /// @param[out] model_coefficients the resultant model coefficients
    bool computeModelCoefficients(const std::vector<uint32_t> &samples,
                                  std::vector<float> &model_coefficients) const override {
        // Need 2 samples
        if (samples.size() != 2) {
            RERROR << "SampleConsensusModelEigenLine::computeModelCoefficients "
                      "Invalid set of samples given " << samples.size();
            return false;
        }

        if (isEqual(input_[samples[0]].x, input_[samples[1]].x) &&
            isEqual(input_[samples[0]].y, input_[samples[1]].y) &&
            isEqual(input_[samples[0]].z, input_[samples[1]].z)) {
            return false;
        }

        Eigen::VectorXf tmp_model_coefficients(6);
        tmp_model_coefficients[0] = input_[samples[0]].x;
        tmp_model_coefficients[1] = input_[samples[0]].y;
        tmp_model_coefficients[2] = input_[samples[0]].z;

        tmp_model_coefficients[3] = input_[samples[1]].x - tmp_model_coefficients[0];
        tmp_model_coefficients[4] = input_[samples[1]].y - tmp_model_coefficients[1];
        tmp_model_coefficients[5] = input_[samples[1]].z - tmp_model_coefficients[2];

        tmp_model_coefficients.template tail<3>().normalize();

        model_coefficients.resize(6);
        model_coefficients[0] = tmp_model_coefficients[0];
        model_coefficients[1] = tmp_model_coefficients[1];
        model_coefficients[2] = tmp_model_coefficients[2];
        model_coefficients[3] = tmp_model_coefficients[3];
        model_coefficients[4] = tmp_model_coefficients[4];
        model_coefficients[5] = tmp_model_coefficients[5];
        return true;
    }

    void getDistancesToModel(const std::vector<float> &model_coefficients,
                             std::vector<float> &distances) const override {
        // Needs a valid set of model coefficients
        if (!isModelValid(model_coefficients)) {
            return;
        }

        distances.resize(indices_.size());

        // Obtain the line point and direction
        Eigen::Vector4f line_pt(model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
        Eigen::Vector4f line_dir(model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
        line_dir.normalize();

        // Iterate through the 3d points and calculate the distances from them to the line
        for (size_t i = 0; i < indices_.size(); ++i) {
            // Calculate the distance from the point to the line
            // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
            // Need to estimate sqrt here to keep MSAC and friends general
            const auto &pt = input_[indices_[i]];
            distances[i] = std::sqrt((line_pt - Eigen::Vector4f(pt.x, pt.y, pt.z, 0.)).cross3(line_dir).squaredNorm());
        }
    }

    /// @brief Select all the points which respect the given model coefficients as inliers.
    /// @param[in] model_coefficients the coefficients of a line model that we need to compute distances to\
    /// @param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
    /// @param[out] inliers the resultant model inliers
    void selectWithinDistance(const std::vector<float> &model_coefficients,
                              const float threshold,
                              std::vector<uint32_t> &inliers) override {
        // Needs a valid set of model coefficients
        if (!isModelValid(model_coefficients)) {
            return;
        }

        float sqr_threshold = threshold * threshold;

        uint32_t nr_p = 0;
        inliers.resize(indices_.size());
        error_sqr_dists_.resize(indices_.size());

        // Obtain the line point and direction
        Eigen::Vector4f line_pt(model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
        Eigen::Vector4f line_dir(model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
        line_dir.normalize();

        // Iterate through the 3d points and calculate the distances from them to the line
        for (size_t i{0}; i < indices_.size(); ++i) {
            // Calculate the distance from the point to the line
            // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
            const auto &pt = input_[indices_[i]];
            float sqr_distance = (line_pt - Eigen::Vector4f(pt.x, pt.y, pt.z, 0.)).cross3(line_dir).squaredNorm();

            if (sqr_distance < sqr_threshold) {
                // Returns the indices of the points whose squared distances are smaller than the threshold
                inliers[nr_p] = indices_[i];
                error_sqr_dists_[nr_p] = sqr_distance;
                ++nr_p;
            }
        }
        inliers.resize(nr_p);
        error_sqr_dists_.resize(nr_p);
    }

    /// @brief Count all the points which respect the given model coefficients as inliers.
    /// @param[in] model_coefficients the coefficients of a model that we need to compute distances to
    /// @param[in] threshold maximum admissible distance threshold for determining the inliers from the outliers
    /// @return the resultant number of inliers
    uint32_t countWithinDistance(const std::vector<float> &model_coefficients,
                                 const float threshold) const override {
        // Needs a valid set of model coefficients
        if (!isModelValid(model_coefficients)) {
            return 0;
        }

        float sqr_threshold = threshold * threshold;

        uint32_t nr_p = 0;

        // Obtain the line point and direction
        Eigen::Vector4f line_pt(model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
        Eigen::Vector4f line_dir(model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
        line_dir.normalize();

        // Iterate through the 3d points and calculate the distances from them to the line
        for (size_t i{0}; i < indices_.size(); ++i) {
            // Calculate the distance from the point to the line
            // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
            const auto &pt = input_[indices_[i]];
            float sqr_distance = (line_pt - Eigen::Vector4f(pt.x, pt.y, pt.z, 0.)).cross3(line_dir).squaredNorm();

            if (sqr_distance < sqr_threshold) {
                nr_p++;
            }
        }
        return nr_p;
    }


    /// @brief Create a new point cloud with inliers projected onto the line model.
    /// @param[in] inliers the data inliers that we want to project on the line model
    /// @param[in] model_coefficients the normalized coefficients of a line model
    /// @param[out] projected_points the resultant projected points
    void projectPoints(const std::vector<uint32_t> &inliers,
                       const std::vector<float> &model_coefficients,
                       std::vector<PointT> &projected_points) const override {
        // Needs a valid model coefficients
        if (!isModelValid(model_coefficients)) {
            return;
        }

        // Obtain the line point and direction
        Eigen::Vector4f line_pt(model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
        Eigen::Vector4f line_dir(model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);

        // Allocate enough space and copy the basics
        projected_points.resize(inliers.size());

        // Iterate over each point
        for (size_t i{0}; i < inliers.size(); ++i) {
            // Iterate over each dimension
            projected_points[i] = input_[inliers[i]];
        }

        // Iterate through the 3d points and calculate the distances from them to the line
        for (size_t i{0}; i < inliers.size(); ++i) {
            const auto &inlier_pt = input_[inliers[i]];
            Eigen::Vector4f pt(inlier_pt.x, inlier_pt.y, inlier_pt.z, 0);
            // k = (DOT_PROD_3D (points[i], p21) - dotA_B) / dotB_B;
            float k = (pt.dot(line_dir) - line_pt.dot(line_dir)) / line_dir.dot(line_dir);

            Eigen::Vector4f pp = line_pt + k * line_dir;
            // Calculate the projection of the point on the line (pointProj = A + k * B)
            projected_points[i].x = pp[0];
            projected_points[i].y = pp[1];
            projected_points[i].z = pp[2];
        }
    }

    /// @brief Verify whether a subset of indices verifies the given line model coefficients.
    /// @param[in] indices the data indices that need to be tested against the line model
    /// @param[in] model_coefficients the line model coefficients
    /// @param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
    bool doSamplesVerifyModel(const std::set<uint32_t> &indices,
                              const std::vector<float> &model_coefficients,
                              const float threshold) const override {
        // Needs a valid set of model coefficients
        if (!isModelValid(model_coefficients)) {
            return false;
        }

        // Obtain the line point and direction
        Eigen::Vector4f line_pt(model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);
        Eigen::Vector4f line_dir(model_coefficients[3], model_coefficients[4], model_coefficients[5], 0);
        line_dir.normalize();

        float sqr_threshold = threshold * threshold;
        // Iterate through the 3d points and calculate the distances from them to the line
        for (const auto &index : indices) {
            // Calculate the distance from the point to the line
            // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
            const auto &pt = input_[index];
            if ((line_pt - Eigen::Vector4f(pt.x, pt.y, pt.z, 0.)).cross3(line_dir).squaredNorm() > sqr_threshold) {
                return false;
            }
        }

        return true;
    }

protected:
    bool isSampleGood(const std::vector<uint32_t> &samples) const override {
        // Make sure that the two sample points are not identical
        if ((input_[samples[0]].x != input_[samples[1]].x) ||
            (input_[samples[0]].y != input_[samples[1]].y) ||
            (input_[samples[0]].z != input_[samples[1]].z)) {
            return true;
        }

        return false;
    }
};

}  // namespace rally

#endif  // RALLY_CORE_ALGORITHM_SAMPLE_CONSENSUS_DETAILS_SAC_MODEL_EIGEN_LINE_H
