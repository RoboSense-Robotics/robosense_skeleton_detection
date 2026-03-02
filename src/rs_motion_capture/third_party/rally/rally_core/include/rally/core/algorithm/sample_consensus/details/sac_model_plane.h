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
#ifndef RALLY_CORE_ALGORITHM_SAMPLE_CONSENSUS_DETAILS_RANSAC_PLANE_H
#define RALLY_CORE_ALGORITHM_SAMPLE_CONSENSUS_DETAILS_RANSAC_PLANE_H

#include "rally/utils/utils.h"
#include "rally/core/containers/containers.h"
#include "rally/core/algorithm/sample_consensus/details/sac_model.h"

namespace rally {

template<typename PointT>
class SampleConsensusModelPlane : public SampleConsensusModel<PointT> {
    using SampleConsensusModel<PointT>::model_name_;
    using SampleConsensusModel<PointT>::sample_size_;
    using SampleConsensusModel<PointT>::model_size_;
    using SampleConsensusModel<PointT>::input_;
    using SampleConsensusModel<PointT>::isModelValid;
    using SampleConsensusModel<PointT>::indices_;
    using SampleConsensusModel<PointT>::error_sqr_dists_;
public:
    using Ptr = std::unique_ptr<SampleConsensusModelPlane<PointT> >;

    /// @brief Constructor for base SampleConsensusModelPlane.
    /// @param[in] cloud the input point cloud dataset
    /// @param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
    explicit SampleConsensusModelPlane(const std::vector<PointT> &cloud, bool random = false)
    : SampleConsensusModel<PointT>(cloud, random) {
        model_name_ = "SampleConsensusModelPlane";
        sample_size_ = 3;
        model_size_ = 4;
    }

    /// @brief Constructor for base SampleConsensusModelPlane.
    /// @param[in] cloud the input point cloud dataset
    /// @param[in] indices a vector of point indices to be used from cloud
    /// @param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
    SampleConsensusModelPlane(const std::vector<PointT> &cloud,
                              const std::vector<uint32_t> &indices,
                              bool random = false)
    : SampleConsensusModel<PointT>(cloud, indices, random) {
        model_name_ = "SampleConsensusModelPlane";
        sample_size_ = 3;
        model_size_ = 4;
    }

    /// @brief Empty destructor
    ~SampleConsensusModelPlane() {}

    /// @brief Check whether the given index samples can form a valid line model, compute the model coefficients from
    /// these samples and store them internally in model_coefficients_. The line coefficients are represented by a
    /// point and a line direction
    /// @param[in] samples the point indices found as possible good candidates for creating a valid model
    /// @param[out] model_coefficients the resultant model coefficients
    bool computeModelCoefficients(const std::vector<uint32_t> &samples,
                                  std::vector<float> &model_coefficients) const override {
        // Need 3 samples
        if (static_cast<uint32_t>(samples.size()) != sample_size_) {
            RERROR << "SampleConsensusModelPlane::computeModelCoefficients "
                      "Invalid set of samples given " << samples.size();
            return false;
        }

        const auto &p0 = input_[samples[0]];
        const auto &p1 = input_[samples[1]];
        const auto &p2 = input_[samples[2]];
        ShortArray4f pp0 = ShortArray4f(p0.x, p0.y, p0.z, 0.f);
        ShortArray4f pp1 = ShortArray4f(p1.x, p1.y, p1.z, 0.f);
        ShortArray4f pp2 = ShortArray4f(p2.x, p2.y, p2.z, 0.f);

        // Compute the segment values (in 3d) between p1 and p0
        ShortArray4f p1p0 = pp1 - pp0;
        // Compute the segment values (in 3d) between p2 and p0
        ShortArray4f p2p0 = pp2 - pp0;

        // Avoid some crashes by checking for collinearity here
        ShortArray4f dy1dy2 = p1p0 / p2p0;

        // Check for collinearity
        if (isEqual(dy1dy2[0], dy1dy2[1]) && isEqual(dy1dy2[2], dy1dy2[1])) {
            return false;
        }

        // Compute the plane coefficients from the 3 given points in a straightforward manner
        // calculate the plane normal n = (p2-p1) x (p3-p1) = cross (p2-p1, p3-p1)
        ShortArray4f tmp_mode_coeff;
        tmp_mode_coeff[0] = p1p0[1] * p2p0[2] - p1p0[2] * p2p0[1];
        tmp_mode_coeff[1] = p1p0[2] * p2p0[0] - p1p0[0] * p2p0[2];
        tmp_mode_coeff[2] = p1p0[0] * p2p0[1] - p1p0[1] * p2p0[0];
        tmp_mode_coeff[3] = 0;

        // Normalize
        tmp_mode_coeff.normalize();

        // ... + d = 0
        tmp_mode_coeff[3] = -1 * (tmp_mode_coeff.dot(pp0));

        model_coefficients.resize(4);
        model_coefficients[0] = tmp_mode_coeff[0];
        model_coefficients[1] = tmp_mode_coeff[1];
        model_coefficients[2] = tmp_mode_coeff[2];
        model_coefficients[3] = tmp_mode_coeff[3];
        return true;
    }

    void getDistancesToModel(const std::vector<float> &model_coefficients,
                             std::vector<float> &distances) const override {
        // Needs a valid set of model coefficients
        if (!isModelValid(model_coefficients)) {
            return;
        }

        distances.resize(indices_.size());
        ShortArray4f tmp_coeff(model_coefficients[0], model_coefficients[1], model_coefficients[2],
                               model_coefficients[3]);

        // Iterate through the 2d points and calculate the distances from them to the curve
        for (size_t i = 0; i < indices_.size(); ++i) {
            // Calculate the distance from the point to the plane normal as the dot product
            // D = (P-A).N/|N|
            //distances[i] = fabs (model_coefficients[0] * input_->points[(*indices_)[i]].x +
            //                     model_coefficients[1] * input_->points[(*indices_)[i]].y +
            //                     model_coefficients[2] * input_->points[(*indices_)[i]].z +
            //                     model_coefficients[3]);
            const auto &pt = input_[indices_[i]];
            ShortArray4f tmp_pt = ShortArray4f(pt.x, pt.y, pt.z, 1.f);
            distances[i] = std::abs(tmp_coeff.dot(tmp_pt));
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

        ShortArray4f tmp_coeff(model_coefficients[0], model_coefficients[1], model_coefficients[2],
                               model_coefficients[3]);

        // Iterate through the 2d points and calculate the distances from them to the curve
        for (size_t i = 0; i < indices_.size(); ++i) {
            const auto &pt = input_[indices_[i]];
            ShortArray4f tmp_pt = ShortArray4f(pt.x, pt.y, pt.z, 1.f);
            float distance = std::abs(tmp_coeff.dot(tmp_pt));

            if (distance < threshold) {
                // Returns the indices of the points whose distances are smaller than the threshold
                inliers[nr_p] = indices_[i];
                error_sqr_dists_[nr_p] = distance;
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

        ShortArray4f tmp_coeff(model_coefficients[0], model_coefficients[1], model_coefficients[2],
                               model_coefficients[3]);

        // Iterate through the 2d points and calculate the distances from them to the curve
        for (size_t i = 0; i < indices_.size(); ++i) {
            const auto &pt = input_[indices_[i]];
            ShortArray4f tmp_pt = ShortArray4f(pt.x, pt.y, pt.z, 1.f);

            if (std::abs(tmp_coeff.dot(tmp_pt)) < threshold) {
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

        ShortArray4f mc(model_coefficients[0], model_coefficients[1], model_coefficients[2], 0);

        // normalize the vector perpendicular to the plane...
        mc.normalize();
        // ... and store the resulting normal as a local copy of the model coefficients
        ShortArray4f tmp_mc = mc;
        tmp_mc[3] = model_coefficients[3];

        // Allocate enough space and copy the basics
        projected_points.resize(inliers.size());

        // Iterate through the 3d points and calculate the distances from them to the plane
        for (size_t i = 0; i < inliers.size(); ++i) {
            const auto &pt = input_[inliers[i]];
            ShortArray4f pp(pt.x, pt.y, pt.z, 1.f);
            // use normalized coefficients to calculate the scalar projection
            float distance_to_plane = tmp_mc.dot(pp);

            auto res = pp - mc * distance_to_plane;        // mc[3] = 0, therefore the 3rd coordinate is safe
            projected_points[i].x = res[0];
            projected_points[i].y = res[1];
            projected_points[i].z = res[2];
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

        ShortArray4f tmp_coeff(model_coefficients[0], model_coefficients[1], model_coefficients[2],
                               model_coefficients[3]);

        for (std::set<uint32_t>::const_iterator it = indices.begin(); it != indices.end(); ++it) {
            const auto &pt = input_[*it];
            ShortArray4f tmp_pt = ShortArray4f(pt.x, pt.y, pt.z, 1.f);
            if (std::abs(tmp_coeff.dot(tmp_pt)) > threshold) {
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

#endif  // RALLY_CORE_ALGORITHM_SAMPLE_CONSENSUS_DETAILS_RANSAC_PLANE_H
