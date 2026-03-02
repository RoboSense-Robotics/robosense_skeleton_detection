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

#ifndef HYPER_VISION_INFERENCE_TRT_INFER_IMPL_H
#define HYPER_VISION_INFERENCE_TRT_INFER_IMPL_H

#include <assert.h>
#include <cudnn.h>

#include <cuda_runtime_api.h>

#include "NvInfer.h"
#include "NvInferPlugin.h"
#include "NvInferRuntime.h"
#include "NvOnnxParser.h"
#include "hyper_vision/common/log.h"
#include "hyper_vision/inference/inference.h"
#include "hyper_vision/inference/trt_bindings.h"
#include "hyper_vision/inference/trt_profiler.h"
#include "hyper_vision/inference/trt_utils.h"
#include "hyper_vision/inference/utils.h"
namespace robosense {
namespace inference {

using NvBuilderCfg = nvinfer1::IBuilderConfig;
using NvBuilder = nvinfer1::IBuilder;
using NvNetDef = nvinfer1::INetworkDefinition;
using NvEngine = nvinfer1::ICudaEngine;
using NvHostMemory = nvinfer1::IHostMemory;
using NvExeCtx = nvinfer1::IExecutionContext;
using NvRuntime = nvinfer1::IRuntime;
using NvProfile = nvinfer1::IOptimizationProfile;
using NvNetDefCrtFlag = nvinfer1::NetworkDefinitionCreationFlag;
using NvNetDefCrtFlags = nvinfer1::NetworkDefinitionCreationFlags;

// Logger for TensorRT
class TrtLogger : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char* msg) throw() {
    // suppress info-level message
    if (severity == Severity::kERROR || severity == Severity::kINTERNAL_ERROR) {
      INFER_ERROR << " " << msg;
      INFER_ERROR << "trt_infer: --- detected system info ---";
      INFER_ERROR << "trt_infer: - cuda version:  " << CUDART_VERSION;
      INFER_ERROR << "trt_infer: - cudnn version:  " << CUDNN_VERSION;
      INFER_ERROR << "trt_infer: - TensorRT version: " << NV_TENSORRT_VERSION;
    }
  }
};

enum class GraphFlag : uint8_t {
  kFIRST_FRAME,
  kFAILED,
  kSUCCESS,
};

// struct InferDeleter {
//   template <typename T>
//   void operator()(T* obj) const {
//     if (obj) {
//       obj->destroy();
//     }
//   }
// };

template <typename T>
using nvUniquePtr = std::unique_ptr<T>;

class TrtInferImpl {
 public:
  using Ptr = std::shared_ptr<TrtInferImpl>;

  TrtInferImpl() {
    // BASE_CUDA_CHECK(cudaEventCreate(&start_));
    // BASE_CUDA_CHECK(cudaEventCreate(&end_));
  }
  std::string getModelVersion() const;
  std::map<std::string, std::string> getModelInfos() const;
  float getInferenceLatency() const;
  void getProfilerInfo() const;

  std::string getInputName(int32_t input_index) const;
  std::string getOutputName(int32_t output_index) const;

  int32_t getNumInputs() const;
  int32_t getNumOutputs() const;

  void initCudaStreamWithPriority();
  cudaStream_t getStream() const;
  int getStreamPriority() const;
  void setStream(cudaStream_t stream);

  size_t getInputSize(int32_t input_index) const;
  size_t getInputSize(const std::string& name) const;

  size_t getOutputSize(int32_t output_index) const;
  size_t getOutputSize(const std::string& name) const;

  Dims getInputDims(int32_t input_index);
  Dims getOutputDims(int32_t output_index);

  void* getInputPtr(int32_t input_index, DeviceType device);
  void* getInputPtr(const std::string& name, DeviceType device);

  void* getOutputPtr(int32_t output_index, DeviceType device);
  void* getOutputPtr(const std::string& name, DeviceType device);
  void readModelFromFile(const ModelFileHandler& file_handler);
  void getTensorInfo(TensorInfo& tensorInfo);
  bool setTensorAddress(const std::string& name, void* address);
  void* getTensorAddress(const std::string& name);
  void changeTensorAddress(const std::string& src_name,
                           const std::string& dst_name);
  void changeTensorAddress(const std::string& src_name,
                           const std::string& dst_name, void* src_device_ptr,
                           void* src_host_ptr);
  void forward();
  void forward(const std::vector<void*>& inputs);
  void forward(const std::vector<void*>& inputs,
               const std::vector<void*>& outputs) {
                // TODO
  }
  void transferInputToDevice();
  void enqueue();
  void transferOutputToHost();
  void cudaSteamSync();

  ~TrtInferImpl() {
    BASE_CUDA_CHECK(cudaStreamSynchronize(stream_));
    // cudaEventDestroy(start_);
    // cudaEventDestroy(end_);
  }

  void init(const InferOptions& options);

  void reset() {
    mOptions_ = {};

    mInputBindingSize_.clear();
    mInputBindingName_.clear();
    mInputDims_.clear();

    mOutputBindingSize_.clear();
    mOutputBindingName_.clear();
    mOutputDims_.clear();

    mInputBindingIndices_.clear();
    mOutputBindingIndices_.clear();
    mBindings_.reset();

    mContext_.reset();
    mEngine_.reset();
    mNetwork_.reset();
    mBuilder_.reset();
  }

 protected:
  // Generate engine cache name to save the engine to disk
  std::string genEngineCacheName(const std::string& save_path,
                                 const std::string& onnx_model);

  // Decrypt the model file, will return the
  std::string decryptModelFile(const std::string& model_file);

  // Deserialize the engine cache from file
  // return 0 if succeed
  int8_t deserializeEngine(const std::string& engine_file);

  // Create engine from onnx or encrypt onnx model
  // return 0 if succeed
  void createEngine(const std::string& onnx_model,
                    const std::string& engine_file);

  // Build the trt engine with config provied by infer options
  // return 0 if succeed
  int8_t buildEngine();

  // Parse onnx model file from onnx file, model format should be
  // onnx model or encrypt onnx model
  // return 0 if succeed
  int8_t buildEngineWithOnnx(const std::string& onnx_model,
                             const std::string& engine_file);

  int8_t setUpBindings();


  // Initialize the engine resources. E.g. allocate the host or device memory
  // return 0 if succeed.
  int8_t setUpInference();

  // Save engine to cache engine file
  // return 0 if succeed.
  int8_t saveEngine(const std::string& file_name);

 protected:
  TrtLogger mLogger_;

  InferOptions mOptions_;

  NvNetDefCrtFlags mFlags_ = 0;

  nvUniquePtr<NvBuilder> mBuilder_;

  nvUniquePtr<NvNetDef> mNetwork_;

  nvUniquePtr<NvRuntime> runtime;

  std::shared_ptr<NvEngine> mEngine_;

  std::shared_ptr<NvHostMemory> mEngineData_;

  nvUniquePtr<NvExeCtx> mContext_;

  // NvProfile *mProfiler_{nullptr};
  Profiler* mProfiler{nullptr};

  std::string model_;
  std::string model_file_name_;
  std::string engine_file_;
  ModelFormat model_format_;

  void** buffers_{nullptr};

  Bindings mBindings_;

  float latency_;

  cudaEvent_t start_, end_;
  cudaGraph_t mGraph_{};
  cudaGraphExec_t mGraphExec_{};
  GraphFlag graphFlag_{GraphFlag::kFIRST_FRAME};
  cudaStream_t stream_;
  int stream_priority_;

  std::vector<TensorInfo> mTensorInfos_;
  size_t mNumInput_{0};
  std::vector<size_t> mInputBindingSize_;
  std::vector<std::string> mInputBindingName_;
  std::vector<Dims> mInputDims_;

  std::vector<size_t> mOutputBindingSize_;
  std::vector<std::string> mOutputBindingName_;
  std::vector<Dims> mOutputDims_;

  std::vector<int32_t> mInputBindingIndices_;
  std::vector<int32_t> mOutputBindingIndices_;

  std::map<std::string, std::string> mModelInfo_;

  std::string version_{"1.0.0"};

};

}  // namespace inference
}  // namespace robosense

#endif
