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
#ifndef HYPER_VISION_INFERENCE_ENTROPY_CALIBRATOR_H
#define HYPER_VISION_INFERENCE_ENTROPY_CALIBRATOR_H

#include <cuda_runtime_api.h>
#include <iterator>
#include "hyper_vision/inference/trt_quantization/batch_stream.h"
#include "NvInfer.h"

namespace robosense {
namespace inference {

//! \class EntropyCalibratorImpl
//!
//! \brief Implements common functionality for Entropy calibrators.
//!
template <typename TBatchStream>
class EntropyCalibratorImpl {
public:
    EntropyCalibratorImpl(
        TBatchStream stream, int firstBatch, std::string TableName, bool readCache = true)
        : mStream{stream}
        , mCalibrationTableName(TableName)
        , mReadCache(readCache) {
        nvinfer1::Dims imageDims = mStream.getImageDims();
        mInputCount = volume(imageDims) * mStream.getBatchSize();

        BASE_CUDA_CHECK(cudaMalloc(&mDeviceInput, mInputCount * sizeof(float)));
        mStream.reset(firstBatch);
    }

    virtual ~EntropyCalibratorImpl() {
        BASE_CUDA_CHECK(cudaFree(mDeviceInput));
    }

    int getBatchSize() const { return mStream.getBatchSize(); }

    bool getBatch(void* bindings[], const char* names[], int nbBindings) {
        if (!mStream.next()) {
            return false;
        }
        BASE_CUDA_CHECK(cudaMemcpy(mDeviceInput, mStream.getBatch(),
                        mInputCount * sizeof(float), cudaMemcpyHostToDevice));
        // assert(!strcmp(names[0], mInputBlobName));
        bindings[0] = mDeviceInput;
        return true;
    }

    const void* readCalibrationCache(size_t& length) {
        mCalibrationCache.clear();
        std::ifstream input(mCalibrationTableName, std::ios::binary);
        input >> std::noskipws;
        if (mReadCache && input.good()) {
            std::copy(std::istream_iterator<char>(input), std::istream_iterator<char>(),
                      std::back_inserter(mCalibrationCache));
        }
        length = mCalibrationCache.size();
        return length ? mCalibrationCache.data() : nullptr;
    }

    void writeCalibrationCache(const void* cache, size_t length) {
        std::ofstream output(mCalibrationTableName, std::ios::binary);
        output.write(reinterpret_cast<const char*>(cache), length);
    }

private:
    TBatchStream mStream;
    size_t mInputCount;
    std::string mCalibrationTableName;
    // const char* mInputBlobName;
    bool mReadCache{true};
    void* mDeviceInput{nullptr};
    std::vector<char> mCalibrationCache;
};

//! \class Int8EntropyCalibrator2
//!
//! \brief Implements Entropy calibrator 2.
//!  CalibrationAlgoType is kENTROPY_CALIBRATION_2.
//!
template <typename TBatchStream>
class Int8EntropyCalibrator2 : public nvinfer1::IInt8EntropyCalibrator2 {
public:
    Int8EntropyCalibrator2(
        TBatchStream stream, int firstBatch, const char* TableName, bool readCache = true)
        : mImpl(stream, firstBatch, TableName, readCache) {}

    int getBatchSize() const noexcept override { return mImpl.getBatchSize(); }

    bool getBatch(void* bindings[], const char* names[], int nbBindings) noexcept override {
        return mImpl.getBatch(bindings, names, nbBindings);
    }

    const void* readCalibrationCache(size_t& length) noexcept override {
        return mImpl.readCalibrationCache(length);
    }

    void writeCalibrationCache(const void* cache, size_t length) noexcept override {
        mImpl.writeCalibrationCache(cache, length);
    }

private:
    EntropyCalibratorImpl<TBatchStream> mImpl;
};


nvinfer1::IInt8Calibrator* getInt8Calibrator(const nvinfer1::Dims4& batchDims,
                                             const std::string& data_path,
                                             const std::string& calibration_table) {
    RSLidarBatchStream calibration_stream(batchDims, data_path);
    return new Int8EntropyCalibrator2<RSLidarBatchStream>(calibration_stream, 0, calibration_table.c_str());
}

}  //  namespace inference
}  //  namespace robosense
#endif  //  ENTROPY_CALIBRATOR_H

