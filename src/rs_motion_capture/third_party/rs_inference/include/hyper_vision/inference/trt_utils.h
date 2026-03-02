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

#ifndef HYPER_VISION_INFERENCE_TRT_UTILS_H
#define HYPER_VISION_INFERENCE_TRT_UTILS_H

#include <dlfcn.h>

#include <cstring>
#include <numeric>

#include <cuda_runtime_api.h>

#include "NvInfer.h"
#include "hyper_vision/common/log.h"
#include "hyper_vision/inference/inference.h"
namespace robosense {
namespace inference {

#define BASE_CUDA_CHECK(condition) \
  { GPUAssert((condition), __FILE__, __LINE__); }

inline void GPUAssert(cudaError_t code, const char* file, int line,
                      bool abort = true) {
  if (code != cudaSuccess) {
    INFER_ERROR << "GPUassert: " << cudaGetErrorString(code) << " " << file
                << " " << line;
    if (abort) {
      exit(code);
    }
  }
}

inline void loadLibrary(const std::string& path) {
  int32_t flags{RTLD_LAZY};
  void* handle = dlopen(path.c_str(), flags);

  if (handle == nullptr) {
    INFER_ERROR << "Could not load plugin library: " << path
                << ", due to: " << dlerror() << std::endl;
  } else {
    INFER_DEBUG << "Load plugin lib " << path;
  }
}

inline size_t dataTypeSize(nvinfer1::DataType dataType) {
  switch (dataType) {
    case nvinfer1::DataType::kINT32:
    case nvinfer1::DataType::kFLOAT:
      return 4U;
    case nvinfer1::DataType::kHALF:
      return 2U;
    case nvinfer1::DataType::kBOOL:
    case nvinfer1::DataType::kUINT8:
    case nvinfer1::DataType::kINT8:
      return 1U;
  }
  return 0;
}

inline int volume(const nvinfer1::Dims& d) {
  return std::accumulate(d.d, d.d + d.nbDims, 1, std::multiplies<int>());
}

inline Dims toDims(const nvinfer1::Dims& d) {
  Dims rd;
  rd.nbDims = d.nbDims;
  std::memcpy(rd.d, d.d, d.nbDims * sizeof(int32_t));
  return rd;
}
template <typename T>
inline T roundUp(T m, T n) {
  return ((m + n - 1) / n) * n;
}

struct TensorInfo {
  int32_t bindingIndex{-1};
  char const* name{nullptr};
  nvinfer1::Dims dims{};
  bool isDynamic{};
  int32_t comps{-1};
  nvinfer1::Dims strides{};
  int32_t vectorDimIndex{-1};
  bool isInput{};
  nvinfer1::DataType dataType{};
  int64_t vol{-1};
  void updateVolume() { vol = volume(dims); }
};

}  // namespace inference
}  // namespace robosense

#endif
