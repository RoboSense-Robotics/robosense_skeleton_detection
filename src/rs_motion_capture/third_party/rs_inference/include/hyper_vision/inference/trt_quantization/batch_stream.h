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
#ifndef HYPER_VISION_INFERENCE_BATCH_STREAM_H
#define HYPER_VISION_INFERENCE_BATCH_STREAM_H

#include <assert.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <numeric>
#include <memory>
#include "hyper_vision/inference/utils.h"
#include "hyper_vision/inference/trt_utils.h"
#include "hyper_vision/common/log.h"

namespace robosense {
namespace inference {

inline std::string locateFile(const std::string& file_name, const std::string& dir) {
        const int MAX_DEPTH{1};
        bool found{false};
        std::string filepath;

        if (!dir.empty() && dir.back() != '/') {
            filepath = dir + "/" + file_name;
        } else {
            filepath = dir + file_name;
        }

        for (int i = 0; i < MAX_DEPTH && !found; i++) {
            std::ifstream checkFile(filepath);
            found = checkFile.is_open();
            if (found) break;
        }
        if (!found) {
            INFER_ERROR << "Could not find " << file_name << "in data directory";
            exit(kFAILURE);
        }
        return filepath;
}

class IBatchStream {
public:
    virtual void reset(int firstBatch) = 0;
    virtual bool next() = 0;
    virtual void skip(int skipCount) = 0;
    virtual float* getBatch() = 0;
    virtual int getBatchesRead() const = 0;
    virtual int getBatchSize() const = 0;
    virtual nvinfer1::Dims getDims() const = 0;
    virtual nvinfer1::Dims getImageDims() const = 0;
};

class RSLidarBatchStream : public IBatchStream {
public:
/**
 * @brief Construct a new RSLidarBatchStream object
 * 
 * @param directory the directory which the calibration dataset stores
 * @param batchDims batchDims is a 4-dims Dimensions which indicate {bath_size, channel, height, width}
 */
    RSLidarBatchStream(nvinfer1::Dims batchDims, const std::string& dir) : mBatchSize_{batchDims.d[0]},
                                                                           mDims_{batchDims},
                                                                           mDataDir_{dir} {
        mListFile_ = getFileInDIR(mDataDir_, ".bin");
        mMaxBatches_ = mListFile_.size() / batchDims.d[0];
        mFeatureSize_ = mDims_.d[1] * mDims_.d[2] * mDims_.d[3];
        mBatch_.resize(mBatchSize_ * mFeatureSize_, 0);
        mFileBatch_.resize(mDims_.d[0] * mFeatureSize_, 0);
        reset(0);
    }

    void reset(int firstBatch) override {
        mBatchCount_ = 0;
        mFileCount_ = 0;
        mFileBatchPos_ = mDims_.d[0];
        skip(firstBatch);
    }

    bool next() override {
        if (mBatchCount_ == mMaxBatches_) {
            return false;
        }
        for (int csize = 1, batchPos = 0; batchPos < mBatchSize_; batchPos += csize, mFileBatchPos_ += csize) {
            assert(mFileBatchPos_ > 0 && mFileBatchPos_ <= mDims_.d[0]);
            if (mFileBatchPos_ == mDims_.d[0] && !update()) {
                return false;
            }
            csize = std::min(mBatchSize_ - batchPos, mDims_.d[0] - mFileBatchPos_);
            std::copy_n(getFileBatch() + mFileBatchPos_ * mFeatureSize_, csize * mFeatureSize_,
                        getBatch() + batchPos * mFeatureSize_);
        }
        mBatchCount_++;
        return true;
    }

    void skip(int skipCount) override {
        if (mBatchSize_ >= mDims_.d[0] && mBatchSize_ % mDims_.d[0] == 0 && mFileBatchPos_ == mDims_.d[0]) {
            mFileCount_ += skipCount * mBatchSize_ / mDims_.d[0];
            return;
        }

        int x = mBatchCount_;
        for (int i = 0; i < skipCount; i++) {
            next();
        }
        mBatchCount_ = x;
    }

    float* getBatch() override {
        return mBatch_.data();
    }

    int getBatchesRead() const override {
        return mBatchCount_;
    }

    int getBatchSize() const override {
        return mBatchSize_;
    }

    nvinfer1::Dims getDims() const override {
        return mDims_;
    }

    nvinfer1::Dims getImageDims() const override {
        return nvinfer1::Dims3{mDims_.d[1], mDims_.d[2], mDims_.d[3]};
    }

    float* getFileBatch() {
        return mFileBatch_.data();
    }

    bool update() {
        if (mListFile_.empty()) {
            INFER_ERROR << "Data list for calibration is empty!";
            exit(kFAILURE);
        } else {
            std::vector<std::string> fNames;
            INFER_DEBUG << "Batch #(" << mFileCount_ << "/" << mMaxBatches_ << ")";
            mFilePos_ = mBatchCount_ * mBatchSize_;
            for (int i = 0; i < mBatchSize_; i++) {
                std::string sName = mListFile_[mFilePos_ + i];
                INFER_DEBUG << "Calibrating with file " << sName;
                fNames.emplace_back(sName);
            }
            mFileCount_++;

            pcd_data.resize(fNames.size());
            for (uint32_t i = 0; i < fNames.size(); ++i) {
                pcd_data[i].clear();
                pcd_data[i].resize(mFeatureSize_, 0);
                std::string filepath = locateFile(fNames[i], mDataDir_);
                readBinFile(filepath, pcd_data[i].data());
            }

            std::vector<float> data(volume(mDims_));
            for (int i = 0; i < mBatchSize_; ++i) {
                std::move(pcd_data[i].begin(), pcd_data[i].begin() + mFeatureSize_, getFileBatch() + i * mFeatureSize_);
            }
        }
        pcd_data.clear();
        mFileBatchPos_ = 0;
        return true;
    }

    void readBinFile(const std::string& filename, float* feature_data) {
        std::ifstream f(filename, std::ios::binary | std::ios::in);
        if (!f) {
            INFER_ERROR << "Can't open  " << filename;
            exit(kFAILURE);
        }
        float* ptr = feature_data;
        for (int i = 0; i < mFeatureSize_; i++) {
            f.read(reinterpret_cast<char *>(&ptr[i]), sizeof(float));
        }
        f.close();
    }

private:
    int mBatchSize_{0};
    int mBatchCount_{0};
    int mMaxBatches_{0};
    int mFileCount_{0};
    int mFilePos_{0};
    int mFileBatchPos_{0};
    int mFeatureSize_{0};
    int mFileNameLength_{0};
    std::vector<std::vector<float>> pcd_data;
    nvinfer1::Dims mDims_;
    std::vector<float> mBatch_;           //  Data for the batch
    std::vector<float> mFileBatch_;       //  List of pcd files
    std::vector<std::string> mListFile_;  //  File name of the list of pcd names
    std::string mDataDir_;                //  Directories where the files can be found
};

}  // namespace inference
}  // namespace robosense

#endif
