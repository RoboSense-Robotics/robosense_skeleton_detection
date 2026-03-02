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
#ifndef RALLY_CORE_ALGORITHM_SAMPLE_CONSENSUS_DETAILS_SAC_MODEL_H
#define RALLY_CORE_ALGORITHM_SAMPLE_CONSENSUS_DETAILS_SAC_MODEL_H

#include <random>
#include "rally/utils/utils.h"
#include "rally/core/meta/contains_fields.h"

namespace rally {

template<typename PointT>
class SampleConsensusModel {
    RALLY_STATIC_ASSERT(details::has_x<PointT>::value &&details::has_y<PointT>::value && details::has_z<PointT>::value);
public:
    using Ptr = std::shared_ptr<SampleConsensusModel<PointT> >;
    /// @brief Empty constructor for base SampleConsensusModel.
    /// @param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
    explicit SampleConsensusModel(bool random = false) : input_{} {
        rng_dist_ = std::make_unique<std::uniform_int_distribution<uint32_t> >(0, std::numeric_limits<int>::max());
        if (random) {
            rng_alg_.seed(static_cast<unsigned> (std::time(nullptr)));
        } else {
            rng_alg_.seed(12345u);
        }
    }

    /// @brief Constructor for base SampleConsensusModel.
    /// @param[in] cloud the input point cloud dataset
    /// @param[in] random if true set the random seed to the current time, else set to 12345 (default: false)
    explicit SampleConsensusModel(const std::vector<PointT> &cloud, bool random = false)
    : input_{} {
        rng_dist_ = std::make_unique<std::uniform_int_distribution<uint32_t> >(0, std::numeric_limits<int>::max());
        if (random) {
            rng_alg_.seed(static_cast<unsigned> (std::time(nullptr)));
        } else {
            rng_alg_.seed(12345u);
        }

        // Sets the input cloud and creates a vector of "fake" indices
        setInputCloud(cloud);
    }

    /// @brief Constructor for base SampleConsensusModel.
    /// @param[in] cloud the input point cloud dataset
    /// @param[in] indices a vector of point indices to be used from cloud
    /// @param[in] random if true set the random seed to the current time, else set to 12345 (default: true)
    SampleConsensusModel(const std::vector<PointT> &cloud,
                         const std::vector<uint32_t> &indices,
                         bool random = true)
    : input_{}, indices_{} {
        rng_dist_ = std::make_unique<std::uniform_int_distribution<uint32_t> >(0, std::numeric_limits<int>::max());
        if (random)
            rng_alg_.seed(static_cast<unsigned> (std::time(nullptr)));
        else
            rng_alg_.seed(12345u);

        if (indices.size() > cloud.size()) {
            RERROR << "Invalid index vector given with size " << indices.size()
                   << " while the input PointCloud has size " << cloud.size() << " !";
        } else {
            indices_ = indices;
        }
        setInputCloud(cloud);
    }

    /// @brief Destructor for base SampleConsensusModel.
    virtual ~SampleConsensusModel() {}

    virtual void getSamples(uint32_t &iterations, std::vector<uint32_t> &samples) noexcept {
        // We're assuming that indices_ have already been set in the constructor
        if (indices_.size() < getSampleSize()) {
            RERROR << "Can not select " << samples.size() << " unique points out of " << indices_.size();
            // one of these will make it stop :)
            samples.clear();
            iterations = std::numeric_limits<uint32_t>::max() - 1;
            return;
        }

        // Get a second point which is different than the first
        samples.resize(getSampleSize());
        for (uint32_t iter = 0; iter < max_sample_checks_; ++iter) {
            // Choose the random indices
            drawIndexSample(samples);

            // If it's a good sample, stop here
            if (isSampleGood(samples)) {
                return;
            }
        }
        samples.clear();
    }

    /// @brief Check whether the given index samples can form a valid model, compute the model coefficients from these samples and store them in model_coefficients. Pure virtual.
    /// @param[in] samples the point indices found as possible good candidates for creating a valid model
    /// @param[out] model_coefficients the computed model coefficients
    virtual bool computeModelCoefficients(const std::vector<uint32_t> &samples,
                                          std::vector<float> &model_coefficients) const = 0;

    /// @brief Compute all distances from the cloud data to a given model. Pure virtual.
    /// @param[in] model_coefficients the coefficients of a model that we need to compute distances to
    /// @param[out] distances the resultant estimated distances
    virtual void getDistancesToModel(const std::vector<float> &model_coefficients,
                                     std::vector<float> &distances) const = 0;

    /// @brief elect all the points which respect the given model coefficients as inliers. Pure virtual.
    /// @param[in] model_coefficients the coefficients of a model that we need to compute distances to
    /// @param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
    /// @param[out] inliers the resultant model inliers
    virtual void selectWithinDistance(const std::vector<float> &model_coefficients, const float threshold,
                                      std::vector<uint32_t> &inliers) = 0;

    /// @brief Count all the points which respect the given model coefficients as inliers. Pure virtual.
    /// @param[in] model_coefficients the coefficients of a model that we need to compute distances to
    /// @param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
    /// @return the resultant number of inliers
    virtual uint32_t countWithinDistance(const std::vector<float> &model_coefficients,
                                         const float threshold) const = 0;

    /// @brief Create a new point cloud with inliers projected onto the model. Pure virtual.
    /// @param[in] inliers the data inliers that we want to project on the model
    /// @param[in] model_coefficients the coefficients of a model
    /// @param[out] projected_points the resultant projected points
    virtual void projectPoints(const std::vector<uint32_t> &inliers,
                               const std::vector<float> &model_coefficients,
                               std::vector<PointT> &projected_points) const = 0;

    /// @brief Verify whether a subset of indices verifies a given set of model coefficients. Pure virtual.
    /// @param[in] indices the data indices that need to be tested against the model
    /// @param[in] model_coefficients the set of model coefficients
    /// @param[in] threshold a maximum admissible distance threshold for determining the inliers from the outliers
    virtual bool doSamplesVerifyModel(const std::set<uint32_t> &indices,
                                      const std::vector<float> &model_coefficients,
                                      const float threshold) const = 0;

    /// @brief input point cloud dataset.
    /// @param[in]  cloud the const boost shared pointer to a PointCloud message
    virtual void setInputCloud(const std::vector<PointT> &cloud) noexcept {
        input_ = cloud;
        if (indices_.empty()) {
            // Prepare a set of indices to be used (entire cloud)
            indices_.resize(cloud.size());
            std::iota(indices_.begin(), indices_.end(), 0);
        }
        shuffled_indices_ = indices_;
    }

    /// @brief Get the input point cloud dataset.
    std::vector<PointT> getInputCloud() const noexcept { return input_; }

    /// @brief Provide the vector of indices that represents the input data.
    /// @param[in] the vector of indices that represents the input data.
    void setIndices(const std::vector<uint32_t> &indices) noexcept {
        indices_ = indices;
        shuffled_indices_ = indices_;
    }

    /// @brief Get the vector of indices used.
    std::vector<uint32_t> getIndices() const noexcept { return indices_; }

    const std::string &getClassName() const noexcept {
        return (model_name_);
    }

    /// @brief Return the size of a sample from which the model is computed.
    uint32_t getSampleSize() const noexcept { return sample_size_; }

    /// @brief Return the number of coefficients in the model.
    uint32_t getModelSize() const noexcept { return model_size_; }

    /// @brief Compute the variance of the errors to the model.
    /// @param[in] error_sqr_dists a vector holding the distances
    float computeVariance(const std::vector<float> &error_sqr_dists) const noexcept {
        std::vector<float> dists(error_sqr_dists);
        const size_t medIdx = dists.size() >> 1;
        std::nth_element(dists.begin(), dists.begin() + medIdx, dists.end());
        float median_error_sqr = dists[medIdx];
        return (2.1981f * median_error_sqr);
    }

    float computeVariance() const noexcept {
        if (error_sqr_dists_.empty()) {
            RERROR << "The variance of the Sample Consensus model distances cannot be estimated, as the model has not been computed yet. Please compute the model first or at least run selectWithinDistance before continuing. Returning NAN!";
            return (std::numeric_limits<float>::quiet_NaN());
        }
        return (computeVariance(error_sqr_dists_));
    }

protected:
    /// @brief Fills a sample array with random samples from the indices_ vector
    /// @param[out] sample the set of indices of target_ to analyze
    void drawIndexSample(std::vector<uint32_t> &sample) noexcept {
        size_t sample_size = sample.size();
        size_t index_size = shuffled_indices_.size();
        for (size_t i = 0; i < sample_size; ++i) {
            // The 1/(RAND_MAX+1.0) trick is when the random numbers are not uniformly distributed and for small modulo
            // elements, that does not matter (and nowadays, random number generators are good)
            // std::swap (shuffled_indices_[i], shuffled_indices_[i + (rand () % (index_size - i))]);
            std::swap(shuffled_indices_[i], shuffled_indices_[i + (rnd() % (index_size - i))]);
        }
        std::copy(shuffled_indices_.begin(), shuffled_indices_.begin() + sample_size, sample.begin());
    }

    /// @brief Check whether a model is valid given the user constraints.
    ///
    /// Default implementation verifies that the number of coefficients in the supplied model is as expected for this
    /// SAC model type. Specific SAC models should extend this function by checking the user constraints (if any).
    ///
    /// @param[in] model_coefficients the set of model coefficients
    virtual bool isModelValid(const std::vector<float> &model_coefficients) const noexcept {
        if (model_coefficients.size() != model_size_) {
            RERROR << getClassName() << " : Invalid number of model coefficients given!";
            return false;
        }
        return true;
    }

    /// @brief Check if a sample of indices results in a good sample of points indices. Pure virtual.
    /// @param[in] samples the resultant index samples
    virtual bool isSampleGood(const std::vector<uint32_t> &samples) const = 0;

    /// @brief The model name.
    std::string model_name_;

    /// @brief point cloud data array.
    std::vector<PointT> input_;

    /// @brief vector of point indices to use.
    std::vector<uint32_t> indices_;

    /// @brief The maximum number of samples to try until we get a good one
    static const uint32_t max_sample_checks_ = 1000;

    /// @brief Data containing a shuffled version of the indices. This is used and modified when drawing samples.
    std::vector<uint32_t> shuffled_indices_;

    /// @brief random number generator algorithm.
    std::mt19937 rng_alg_;

    /// @brief random number generator distribution.
    std::unique_ptr<std::uniform_int_distribution<uint32_t> > rng_dist_;

    /// @brief A vector holding the distances to the computed model. Used internally.
    std::vector<float> error_sqr_dists_;

    /// @brief The size of a sample from which the model is computed. Every subclass should initialize this appropriately.
    uint32_t sample_size_;

    /// @brief The number of coefficients in the model. Every subclass should initialize this appropriately.
    uint32_t model_size_;

    /// @brief random number generator.
    uint32_t rnd() noexcept { return ((*rng_dist_)(rng_alg_)); }
};

}  // namespace rally

#endif  // RALLY_CORE_ALGORITHM_SAMPLE_CONSENSUS_DETAILS_SAC_MODEL_H
