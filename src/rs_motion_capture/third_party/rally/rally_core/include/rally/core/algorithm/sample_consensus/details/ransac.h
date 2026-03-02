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
#ifndef RALLY_CORE_ALGORITHM_SAMPLE_CONSENSUS_DETAILS_RANSAC_H
#define RALLY_CORE_ALGORITHM_SAMPLE_CONSENSUS_DETAILS_RANSAC_H

#include <ctime>
#include <memory>
#include <set>
#include "rally/core/algorithm/sample_consensus/details/sac_model.h"

namespace rally {

/// @brief RandomSampleConsensus represents an implementation of the RANSAC (RAndom SAmple Consensus) algorithm, as
/// described in: "Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and
/// Automated Cartography", Martin A. Fischler and Robert C. Bolles, Comm. Of the ACM 24: 381–395, June 1981.
template<typename PointT>
class RandomSampleConsensus {
    using SampleConsensusModelPtr = typename SampleConsensusModel<PointT>::Ptr;
public:
    using Ptr = std::shared_ptr<RandomSampleConsensus<PointT> >;

    /// @brief RANSAC (RAndom SAmple Consensus) main constructor
    /// @param[in] model a Sample Consensus model
    /// @param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
    explicit RandomSampleConsensus(const SampleConsensusModelPtr &model, bool random = false)
    : sac_model_ptr_(model), probability_(0.99), iterations_(0), threshold_(std::numeric_limits<float>::max()),
      max_iterations_(1000) {
        rng_ptr_ = std::make_shared<std::uniform_real_distribution<float> >(0., 1.);
        if (random) {
            rng_alg_.seed(std::time(nullptr));
        } else {
            rng_alg_.seed(12345u);
        }
    }

    /// @brief RANSAC (RAndom SAmple Consensus) main constructor
    /// @param[in] model a Sample Consensus model
    /// @param[in] threshold distance to model threshold
    /// @param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
    RandomSampleConsensus(const SampleConsensusModelPtr &model, float threshold, bool random = false)
    : sac_model_ptr_(model), probability_(0.99), iterations_(0), threshold_(threshold), max_iterations_(1000) {
        rng_ptr_ = std::make_shared<std::uniform_real_distribution<float> >(0., 1.);
        // Create a random number generator object
        if (random) {
            rng_alg_.seed(std::time(nullptr));
        } else {
            rng_alg_.seed(12345u);
        }
    }

    /// @brief Set the Sample Consensus model to use.
    /// @param[in] model a Sample Consensus model
    void setSampleConsensusModel(const SampleConsensusModelPtr &model) {
        sac_model_ptr_ = model;
    }

    /// @brief Get the Sample Consensus model used.
    SampleConsensusModelPtr getSampleConsensusModel() const {
        return sac_model_ptr_;
    }

    /// @brief  Destructor for RandomSampleConsensus.
    ~RandomSampleConsensus() {}

    /// @brief Set the distance to model threshold.
    /// @param[in] threshold distance to model threshold
    void setDistanceThreshold(float threshold) { threshold_ = threshold; }

    /// @brief Get the distance to model threshold, as set by the user.
    float getDistanceThreshold() const { return threshold_; }

    /// @brief Set the maximum number of iterations.
    /// @param[in] max_iterations maximum number of iterations
    void setMaxIterations(uint32_t max_iterations) { max_iterations_ = max_iterations; }

    /// @brief Get the maximum number of iterations, as set by the user.
    uint32_t getMaxIterations() const { return max_iterations_; }

    /// @brief Set the desired probability of choosing at least one sample free from outliers.
    /// @param[in] probability the desired probability of choosing at least one sample free from outliers
    /// note internally, the probability is set to 99% (0.99) by default.
    void setProbability(float probability) { probability_ = probability; }

    /// @brief Obtain the probability of choosing at least one sample free from outliers, as set by the user.
    float getProbability() const { return probability_; }

    /// @brief Compute the actual model and find the inliers
    bool computeModel() {
        // Warn and exit if no threshold was set
        if (threshold_ == std::numeric_limits<float>::max()) {
            RERROR << "RandomSampleConsensus::computeModel: No threshold set!";
            return false;
        }

        iterations_ = 0;
        uint32_t n_best_inliers_count = 0;
        float k = 1.f;

        std::vector<uint32_t> selection;
        std::vector<float> model_coefficients;

        float log_probability = std::log(1.f - probability_);
        float one_over_indices = 1.f / static_cast<float> (sac_model_ptr_->getIndices().size());

        uint32_t n_inliers_count = 0;
        uint32_t skipped_count = 0;
        // suppress infinite loops by just allowing 10 x maximum allowed iterations for invalid model parameters!
        const uint32_t max_skip = max_iterations_ * 10;

        // Iterate
        while (iterations_ < k && skipped_count < max_skip) {
            // Get X samples which satisfy the model criteria
            sac_model_ptr_->getSamples(iterations_, selection);

            if (selection.empty()) {
                RERROR << "RandomSampleConsensus::computeModel: No samples could be selected!";
                break;
            }

            // Search for inliers in the point cloud for the current plane model M
            if (!sac_model_ptr_->computeModelCoefficients(selection, model_coefficients)) {
                ++skipped_count;
                continue;
            }

            n_inliers_count = sac_model_ptr_->countWithinDistance(model_coefficients, threshold_);

            // Better match ?
            if (n_inliers_count > n_best_inliers_count) {
                n_best_inliers_count = n_inliers_count;

                // Save the current model/inlier/coefficients selection as being the best so far
                model_ = selection;
                model_coefficients_ = model_coefficients;

                // Compute the k parameter (k=log(z)/log(1-w^n))
                float w = static_cast<float> (n_best_inliers_count) * one_over_indices;
                float p_no_outliers = 1.f - std::pow(w, static_cast<float> (selection.size()));
                p_no_outliers = std::max(std::numeric_limits<float>::epsilon(),
                                         p_no_outliers);       // Avoid division by -Inf
                p_no_outliers = std::min(1.f - std::numeric_limits<float>::epsilon(),
                                         p_no_outliers);   // Avoid division by 0.
                k = log_probability / std::log(p_no_outliers);
            }

            ++iterations_;
            if (iterations_ > max_iterations_) {
                break;
            }
        }

        if (model_.empty()) {
            inliers_.clear();
            return false;
        }

        // Get the set of inliers that correspond to the best model found so far
        sac_model_ptr_->selectWithinDistance(model_coefficients_, threshold_, inliers_);
        return true;
    }

    /// @brief Get a set of randomly selected indices.
    /// @param[in] indices indices the input indices vector
    /// @param[in] nr_samples the desired number of point indices to randomly select
    /// @param[out] indices_subset the resultant output set of randomly selected indices
    void getRandomSamples(const std::vector<uint32_t> &indices, size_t nr_samples, std::set<uint32_t> &indices_subset) {
        indices_subset.clear();
        while (indices_subset.size() < nr_samples) {
            indices_subset.insert(indices[static_cast<uint32_t> (static_cast<float>(indices.size()) * rnd())]);
        }
    }

    /// @brief Return the best model found so far.
    /// @param[out] model the resultant model
    void getModel(std::vector<uint32_t> &model) const { model = model_; }

    /// @brief Return the best set of inliers found so far for this model.
    /// @paramp[out] inliers the resultant set of inliers
    void getInliers(std::vector<uint32_t> &inliers) const { inliers = inliers_; }

    /// @brief Return the model coefficients of the best model found so far.
    /// @param[out] model_coefficients the resultant model coefficients
    void getModelCoefficients(std::vector<float> &model_coefficients) const {
        model_coefficients = model_coefficients_;
    }

private:
    /// @brief The underlying data model used (i.e. what is it that we attempt to search for).
    SampleConsensusModelPtr sac_model_ptr_;

    /// @brief The model found after the last computeModel () as point cloud indices.
    std::vector<uint32_t> model_;

    /// @brief The indices of the points that were chosen as inliers after the last computeModel () call.
    std::vector<uint32_t> inliers_;

    /// @brief The coefficients of our model computed directly from the model found.
    std::vector<float> model_coefficients_;

    /// @brief Desired probability of choosing at least one sample free from outliers.
    float probability_;

    /// @brief Total number of internal loop iterations that we've done so far.
    uint32_t iterations_;

    /// @brief Distance to model threshold.
    float threshold_;

    /// @brief Maximum number of iterations before giving up.
    uint32_t max_iterations_;

    /// @brief std-based random number generator algorithm.
    std::mt19937 rng_alg_;

    /// @brief std-based random number generator distribution.
    std::shared_ptr<std::uniform_real_distribution<float> > rng_ptr_;

    /// @brief std-based random number generator.
    float rnd() {
        return ((*rng_ptr_)(rng_alg_));
    }
};

}  // namespace rally

#endif  // RALLY_CORE_ALGORITHM_SAMPLE_CONSENSUS_DETAILS_RANSAC_H
