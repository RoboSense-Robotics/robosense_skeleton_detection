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
#ifndef RALLY_CORE_ALGORITHM_SAMPLE_CONSENSUS_DETAILS_SAC_MODEL_EIGEN_QUADRIC_CURVE_H
#define RALLY_CORE_ALGORITHM_SAMPLE_CONSENSUS_DETAILS_SAC_MODEL_EIGEN_QUADRIC_CURVE_H

#include <eigen3/Eigen/Dense>
#include "rally/utils/utils.h"
#include "rally/core/containers/containers.h"
#include "rally/core/algorithm/sample_consensus/details/sac_model.h"
#include "rally/core/algorithm/geometry/polynomial_fitting.h"

namespace rally {

template<typename PointT>
class SampleConsensusModelEigenQuadricCurve : public SampleConsensusModel<PointT> {
    using SampleConsensusModel<PointT>::model_name_;
    using SampleConsensusModel<PointT>::sample_size_;
    using SampleConsensusModel<PointT>::model_size_;
    using SampleConsensusModel<PointT>::input_;
    using SampleConsensusModel<PointT>::isModelValid;
    using SampleConsensusModel<PointT>::indices_;
    using SampleConsensusModel<PointT>::error_sqr_dists_;
public:
    using Ptr = std::unique_ptr<SampleConsensusModelEigenQuadricCurve<PointT> >;

    /// @brief Constructor for base SampleConsensusModelQuadricCurve.
    /// @param[in] cloud the input point cloud dataset
    /// @param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
    explicit SampleConsensusModelEigenQuadricCurve(const std::vector<PointT> &cloud, bool random = false)
    : SampleConsensusModel<PointT>(cloud, random) {
        model_name_ = "SampleConsensusModelEigenQuadricCurve";
        sample_size_ = 3;
        model_size_ = 3;
    }

    /// @brief Constructor for base SampleConsensusModelQuadricCurve.
    /// @param[in] cloud the input point cloud dataset
    /// @param[in] indices a vector of point indices to be used from cloud
    /// @param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
    SampleConsensusModelEigenQuadricCurve(const std::vector<PointT> &cloud,
                                          const std::vector<uint32_t> &indices,
                                          bool random = false)
    : SampleConsensusModel<PointT>(cloud, indices, random) {
        model_name_ = "SampleConsensusModelEigenQuadricCurve";
        sample_size_ = 3;
        model_size_ = 3;
    }

    /// @brief Empty destructor
    ~SampleConsensusModelEigenQuadricCurve() {}

    /// @brief Check whether the given index samples can form a valid line model, compute the model coefficients from
    /// these samples and store them internally in model_coefficients_. The line coefficients are represented by a
    /// point and a line direction
    /// @param[in] samples the point indices found as possible good candidates for creating a valid model
    /// @param[out] model_coefficients the resultant model coefficients
    bool computeModelCoefficients(const std::vector<uint32_t> &samples,
                                  std::vector<float> &model_coefficients) const override {
        // Need 3 samples
        if (static_cast<uint32_t>(samples.size()) != sample_size_) {
            RERROR << "SampleConsensusModelEigenQuadricCurve::computeModelCoefficients "
                      "Invalid set of samples given " << samples.size();
            return false;
        }

        if (isEqual(input_[samples[0]].x, input_[samples[1]].x) &&
            isEqual(input_[samples[0]].x, input_[samples[2]].x) &&
            isEqual(input_[samples[1]].x, input_[samples[2]].x)) {
            return false;
        }

        const auto &pt0 = input_[samples[0]];
        const auto &pt1 = input_[samples[1]];
        const auto &pt2 = input_[samples[2]];

        std::vector<ShortArray2f> pts(3);
        pts[0].x = pt0.x;
        pts[0].y = pt0.y;
        pts[1].x = pt1.x;
        pts[1].y = pt1.y;
        pts[2].x = pt2.x;
        pts[2].y = pt2.y;
        auto res = polyfit(pts, 2);

        model_coefficients.resize(model_size_);
        model_coefficients[0] = res[2];
        model_coefficients[1] = res[1];
        model_coefficients[2] = res[0];
        return true;
    }

    void getDistancesToModel(const std::vector<float> &model_coefficients,
                             std::vector<float> &distances) const override {
        // Needs a valid set of model coefficients
        if (!isModelValid(model_coefficients)) {
            return;
        }

        distances.resize(indices_.size());

        // Iterate through the 2d points and calculate the distances from them to the curve
        for (size_t i = 0; i < indices_.size(); ++i) {
            const auto &pt = input_[indices_[i]];
            float model_y = model_coefficients[0] * pt.x * pt.x + model_coefficients[1] * pt.x + model_coefficients[2];

            distances[i] = std::abs(pt.y - model_y);
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

        uint32_t nr_p = 0;
        inliers.resize(indices_.size());
        error_sqr_dists_.resize(indices_.size());

        // Iterate through the 2d points and calculate the distances from them to the curve
        for (size_t i = 0; i < indices_.size(); ++i) {
            const auto &pt = input_[indices_[i]];
            float model_y = model_coefficients[0] * pt.x * pt.x + model_coefficients[1] * pt.x + model_coefficients[2];
            float dist = std::abs(pt.y - model_y);

            if (dist < threshold) {
                // Returns the indices of the points whose squared distances are smaller than the threshold
                inliers[nr_p] = indices_[i];
                error_sqr_dists_[nr_p] = dist;
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
        uint32_t nr_p = 0;

        // Iterate through the 2d points and calculate the distances from them to the curve
        for (size_t i = 0; i < indices_.size(); ++i) {
            const auto &pt = input_[indices_[i]];
            float model_y = model_coefficients[0] * pt.x * pt.x +
                            model_coefficients[1] * pt.x + model_coefficients[2];
            float dist = std::abs(pt.y - model_y);
            if (dist < threshold) {
                ++nr_p;
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

        // Allocate enough space and copy the basics
        projected_points.resize(inliers.size());

        // Iterate over each point
        for (size_t i = 0; i < inliers.size(); ++i) {
            // Iterate over each dimension
            projected_points[i] = input_[inliers[i]];
        }

        // Iterate through the 3d points and calculate the distances from them to the line
        for (size_t i = 0; i < inliers.size(); ++i) {
            const auto &pt = input_[indices_[i]];
            float model_y = model_coefficients[0] * pt.x * pt.x + model_coefficients[1] * pt.x + model_coefficients[2];

            // Calculate the projection of the point on the line (pointProj = A + k * B)
            projected_points[i].x = pt.x;
            projected_points[i].y = model_y;
            projected_points[i].z = 0.0;
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
        // Iterate through the 3d points and calculate the distances from them to the line
        for (const auto &index : indices) {
            const auto &pt = input_[index];
            float model_y = model_coefficients[0] * pt.x * pt.x + model_coefficients[1] * pt.x + model_coefficients[2];
            float dist = std::abs(pt.y - model_y);

            if (dist > threshold) {
                return false;
            }
        }
        return true;
    }

protected:
    bool isSampleGood(const std::vector<uint32_t> &samples) const override {
        // Make sure that the two sample points are not identical
        if ((input_[samples[0]].x != input_[samples[1]].x) ||
            (input_[samples[1]].x != input_[samples[2]].x) ||
            (input_[samples[1]].x != input_[samples[2]].x)) {
            return true;
        }
        return false;
    }
};

}  // namespace rally

#endif  // RALLY_CORE_ALGORITHM_SAMPLE_CONSENSUS_DETAILS_SAC_MODEL_EIGEN_QUADRIC_CURVE_H
