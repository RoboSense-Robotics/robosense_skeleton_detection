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

#include "hyper_vision/inference/trt_infer_impl.h"

#include <algorithm>
#include <fstream>
#include <thread>
#include <vector>

#include "rs_perception/rs_cryptor.h"
#if !defined(USE_TENSORRT_V8)
#include "hyper_vision/inference/trt_quantization/entropy_calibrator.h"
#endif
#ifndef CMAKE_DEV
#include "nvToolsExt.h"

#include "cyber/common/log.h"
#endif
// using namespace robosense::perception;

namespace robosense {
namespace inference {

std::string TrtInferImpl::getModelVersion() const {
  if (mModelInfo_.empty()) {
    return {};
  } else if (mModelInfo_.find("version") != mModelInfo_.end()) {
    return mModelInfo_.at("version");
  } else {
    INFER_ERROR << "Fail to get model version, please check the model!";
    exit(kFAILURE);
  }
}

std::map<std::string, std::string> TrtInferImpl::getModelInfos() const {
  if (mModelInfo_.empty()) {
    return {};
  } else {
    return mModelInfo_;
  }
}

float TrtInferImpl::getInferenceLatency() const { return latency_; }

void TrtInferImpl::getProfilerInfo() const {
  //    mProfiler->print(std::cout);

  std::string save_path = "/tmp/trt_debug.json";
  mProfiler->exportJSONProfile(save_path);
}

std::string TrtInferImpl::getInputName(int32_t input_index) const {
  return mBindings_.getInputBinding(input_index).name;
}

std::string TrtInferImpl::getOutputName(int32_t output_index) const {
  return mBindings_.getOutputBinding(output_index).name;
}

int32_t TrtInferImpl::getNumInputs() const { return mBindings_.mNumInput_; }

int32_t TrtInferImpl::getNumOutputs() const {
  return mBindings_.size() - mBindings_.mNumInput_;
}

cudaStream_t TrtInferImpl::getStream() const { return stream_; }

void TrtInferImpl::setStream(cudaStream_t stream) { stream_ = stream; }

int TrtInferImpl::getStreamPriority() const { return stream_priority_; }

size_t TrtInferImpl::getInputSize(int32_t input_index) const {
  return mBindings_.getInputBinding(input_index).volume;
}

size_t TrtInferImpl::getInputSize(const std::string& name) const {
  return mBindings_.getBinding(name).volume;
}

size_t TrtInferImpl::getOutputSize(int32_t output_index) const {
  return mBindings_.getOutputBinding(output_index).volume;
}

size_t TrtInferImpl::getOutputSize(const std::string& name) const {
  return mBindings_.getBinding(name).volume;
}

Dims TrtInferImpl::getInputDims(int32_t input_index) {
  if (input_index >= static_cast<int32_t>(mNumInput_)) {
    INFER_ERROR << "Input index should be less than the number of inputs";
    exit(kFAILURE);
  }
  return toDims(mTensorInfos_[input_index].dims);
}

Dims TrtInferImpl::getOutputDims(int32_t output_index) {
  output_index = output_index + mNumInput_;
  if (output_index >= static_cast<int32_t>(mTensorInfos_.size())) {
    INFER_ERROR << "Output index should be less than the number of outputs";
    exit(kFAILURE);
  }
  return toDims(mTensorInfos_[output_index].dims);
}

void* TrtInferImpl::getInputPtr(const int32_t input_index, DeviceType device) {
  const Binding& binding = mBindings_.getInputBinding(input_index);
  switch (device) {
    case DeviceType::kCPU: {
      return binding.buffer->getHostBuffer();
    }
    case DeviceType::kGPU: {
      return binding.buffer->getDeviceBuffer();
    }
    case DeviceType::kDSP: {
      INFER_ERROR << "Unsupport device type!";
      exit(kFAILURE);
    }
    default: {
      INFER_ERROR << "Unknown device type!";
      exit(kFAILURE);
    }
  }
}

void* TrtInferImpl::getInputPtr(const std::string& name, DeviceType device) {
  const Binding& binding = mBindings_.getBinding(name);
  switch (device) {
    case DeviceType::kCPU: {
      return binding.buffer->getHostBuffer();
      break;
    }
    case DeviceType::kGPU: {
      return binding.buffer->getDeviceBuffer();
      break;
    }
    case DeviceType::kDSP: {
      INFER_ERROR << "Unsupport device type now!";
      exit(kFAILURE);
    }
    default: {
      INFER_ERROR << "Unknown device type!";
      exit(kFAILURE);
    }
  }
}

void* TrtInferImpl::getOutputPtr(int32_t output_index, DeviceType device) {
  const Binding& binding = mBindings_.getOutputBinding(output_index);
  switch (device) {
    case DeviceType::kCPU: {
      return binding.buffer->getHostBuffer();
    }
    case DeviceType::kGPU: {
      return binding.buffer->getDeviceBuffer();
    }
    case DeviceType::kDSP: {
      INFER_ERROR << "Unsupport device type!";
      exit(kFAILURE);
    }
    default:
      INFER_ERROR << "Unknown device type!";
      exit(kFAILURE);
  }
}

void* TrtInferImpl::getOutputPtr(const std::string& name, DeviceType device) {
  const Binding& binding = mBindings_.getBinding(name);
  switch (device) {
    case DeviceType::kCPU:
      return binding.buffer->getHostBuffer();
      break;
    case DeviceType::kGPU:
      return binding.buffer->getDeviceBuffer();
    case DeviceType::kDSP: {
      INFER_ERROR << "Unsupport device type!";
      exit(kFAILURE);
    }
    default:
      INFER_ERROR << "Unknown device type!";
      exit(kFAILURE);
  }
  INFER_ERROR << name << " not found in network! Please check the output name!";
  exit(kFAILURE);
}

bool TrtInferImpl::setTensorAddress(const std::string& name, void* address) {
  // address mush be in gpu
  return mContext_->setTensorAddress(name.c_str(), address);
}

void* TrtInferImpl::getTensorAddress(const std::string& name) {
  return const_cast<void*>(mContext_->getTensorAddress(name.c_str()));
}

void TrtInferImpl::changeTensorAddress(const std::string& src_name,
                                       const std::string& dst_name) {
  void* src = getTensorAddress(src_name);
  setTensorAddress(dst_name, src);
  // sync with binding
  const Binding& src_binding = mBindings_.getBinding(src_name);
  const Binding& dst_binding = mBindings_.getBinding(dst_name);
  // if (src_name == "prev_feats_lvl0_T-4") {
  //   INFER_INFO << "BEFORE: " << src_binding.buffer->getHostBuffer();
  //   INFER_INFO << "BEFORE: " << dst_binding.buffer->getHostBuffer();
  // }
  // INFER_INFO << src_name << "  " << src_binding.isInput;
  // INFER_INFO << dst_name << "  " << dst_binding.isInput;
  // INFER_INFO << "========";
  if (dst_binding.isInput && src_binding.isInput) {
    dst_binding.buffer->setDeviceBuffer(src_binding.buffer->getDeviceBuffer());
    dst_binding.buffer->setHostBuffer(src_binding.buffer->getHostBuffer());
    // if (src_name == "prev_feats_lvl0_T-4") {
    // INFER_INFO << " AFTER: " << src_binding.buffer->getHostBuffer();
    // INFER_INFO << " AFTER: " << dst_binding.buffer->getHostBuffer();
    // }
  } else if (dst_binding.isInput && !src_binding.isInput) {
    if (src_binding.outputAllocator != nullptr) {
      dst_binding.buffer->setDeviceBuffer(
          src_binding.outputAllocator->getDeviceBuffer());
      dst_binding.buffer->setHostBuffer(
          src_binding.outputAllocator->getHostBuffer());
    } else {
      dst_binding.buffer->setDeviceBuffer(
          src_binding.buffer->getDeviceBuffer());
      dst_binding.buffer->setHostBuffer(src_binding.buffer->getHostBuffer());
    }
  } else if (!dst_binding.isInput && src_binding.isInput) {
    dst_binding.buffer->setHostBuffer(src_binding.buffer->getHostBuffer());
    if (dst_binding.outputAllocator != nullptr) {
      dst_binding.outputAllocator->setDeviceBuffer(
          src_binding.buffer->getDeviceBuffer());
      dst_binding.outputAllocator->setHostBuffer(
          src_binding.buffer->getHostBuffer());
    } else {
      dst_binding.buffer->setDeviceBuffer(
          src_binding.buffer->getDeviceBuffer());
      dst_binding.buffer->setHostBuffer(src_binding.buffer->getHostBuffer());
    }
  }
}

void TrtInferImpl::changeTensorAddress(const std::string& src_name,
                                       const std::string& dst_name,
                                       void* src_device_ptr,
                                       void* src_host_ptr) {
  // void* src = getTensorAddress(src_name);
  setTensorAddress(dst_name, src_device_ptr);
  // sync with binding
  // INFER_INFO << src_name << " to " << dst_name;

  const Binding& src_binding = mBindings_.getBinding(src_name);
  const Binding& dst_binding = mBindings_.getBinding(dst_name);
  // INFER_INFO << "SRC BEFORE: " << src_binding.buffer->getHostBuffer();
  // INFER_INFO << "DST BEFORE: " << dst_binding.buffer->getHostBuffer();
  dst_binding.buffer->setDeviceBuffer(src_device_ptr);
  dst_binding.buffer->setHostBuffer(src_host_ptr);
  // INFER_INFO << "SRC  AFTER: " << src_binding.buffer->getHostBuffer();
  // INFER_INFO << "DST  AFTER: " << dst_binding.buffer->getHostBuffer();
}

inline int8_t checkPlatform(const InferOptions& options) {
  INFER_DEBUG << "Checking deployment platform...";
  int32_t count{0};
  if (cudaSuccess != cudaGetDeviceCount(&count)) {
    return kFAILURE;
  }
  if (count == 0) {
    return kFAILURE;
  }
  if (options.device_id >= count) {
    INFER_ERROR << "Invalid device id " << options.device_id;
    exit(kFAILURE);
  }
  for (int32_t device = 0; device < count; ++device) {
    cudaDeviceProp prop;
    if (cudaSuccess == cudaGetDeviceProperties(&prop, device)) {
      int32_t sm{prop.major * 10 + prop.minor};
      if (sm >= 86) {
        if (CUDART_VERSION < 11010) {
          INFER_TRACE << "The cuda version " << CUDART_VERSION
                      << " of this platform is "
                      << "incompatiable with the gpu device of sm{}." << sm
                      << " . There may get fault when buiding inference engine";
        }
      } else {
        INFER_TRACE
            << "Please note that there may get fault when running in platform "
            << "with cuda version below" << CUDART_VERSION;
      }
    }
  }
  return kSUCCESS;
}

static void FindLastFileName(const std::string& full_file_path,
                             std::string& model_file_name) {
  std::string::size_type i_pos = full_file_path.find_last_of('/') + 1;
  model_file_name =
      full_file_path.substr(i_pos, full_file_path.length() - i_pos);
  return;
}

void TrtInferImpl::readModelFromFile(const ModelFileHandler& file_handler) {
  model_format_ = file_handler.modelFormat;

  if (!file_handler.engineFile.empty() &&
      model_format_ == ModelFormat::kENGINE) {
    INFER_DEBUG << "Reading engine model file from  "
                << file_handler.engineFile;
    FindLastFileName(file_handler.engineFile, model_file_name_);
    if (pathExists(file_handler.engineFile)) {
      model_ = file_handler.engineFile;
    } else {
      INFER_ERROR << "Engine model file is not found!";
      exit(kFAILURE);
    }
  } else if (!file_handler.onnxFile.empty()) {
    INFER_DEBUG << "Reading onnx model file from  " << file_handler.onnxFile;
    model_ = decryptModelFile(file_handler.onnxFile);
    engine_file_ = file_handler.engineFile;
    FindLastFileName(file_handler.onnxFile, model_file_name_);
  } else {
    INFER_ERROR << "Model file is empty! Please provide the model file path!";
    exit(kFAILURE);
  }
}

void  TrtInferImpl::initCudaStreamWithPriority() {
  // create cudastream
  int priority_low;
  int priority_hi;
  cudaError_t ert =
      cudaDeviceGetStreamPriorityRange(&priority_low, &priority_hi);
  if (ert != cudaSuccess) {
    INFER_ERROR << "the cudaDeviceGetStreamPriorityRange error: "
           << cudaGetErrorString(ert);
    exit(kFAILURE);
  }
  // 数字越小表示优先级越高, orin的优先级为0到-5
  int prio_range = priority_low - priority_hi;
  if (mOptions_.infer_stream_priority > prio_range) {
    mOptions_.infer_stream_priority = prio_range;
  }
  stream_priority_ = priority_low - mOptions_.infer_stream_priority;
  ert = cudaStreamCreateWithPriority(&stream_, cudaStreamNonBlocking, stream_priority_);
  if (ert != cudaSuccess) {
    INFER_ERROR << "the cudaStreamCreateWithPriority error: "
            << cudaGetErrorString(ert);
    int restart_count = 0;
    while (restart_count < 5 && ert != cudaSuccess) {
      ++restart_count;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      INFER_ERROR << "the cudaStreamCreateWithPriority error: "
              << cudaGetErrorString(ert) << " retry for :" << restart_count;
      ert = cudaStreamCreateWithPriority(&stream_, cudaStreamNonBlocking, stream_priority_);
    }
    if (ert != cudaSuccess) {
      exit(kFAILURE);
    }
  }

  if (ert != cudaSuccess) {
    INFER_ERROR << "the cudaStreamCreateWithPriority error: "
           << cudaGetErrorString(ert);
    exit(kFAILURE);
  }
}

void TrtInferImpl::init(const InferOptions& options) {
  std::string build_date = __DATE__;
  std::string build_time = __TIME__;
  INFER_INFO << "rs_inference version:" << version_
             << ", build time: " << build_date << " " << build_time;

  mOptions_ = options;
  initCudaStreamWithPriority();

  int8_t status{checkPlatform(mOptions_)};

  BASE_CUDA_CHECK(cudaSetDevice(mOptions_.device_id));
  initLibNvInferPlugins(&mLogger_, "");
//#ifndef CMAKE_DEV
//  loadLibrary("/opt/apollo/tensorrt/lib/libcustom_trt_plugin.so");
//#else
//#ifdef PROJECT_PATH
//  loadLibrary(std::string(PROJECT_PATH) + "/build/lib/libcustom_trt_plugin.so");
//#else
//  loadLibrary("build/lib/libcustom_trt_plugin.so");
//#endif
//#endif
  if (mOptions_.model_format == ModelFormat::kENGINE) {
    INFER_DEBUG << "Trying to load engine from " << mOptions_.save_path;
    FindLastFileName(mOptions_.save_path, model_file_name_);
    createEngine("", mOptions_.save_path);
  } else if (mOptions_.model_format == ModelFormat::kONNX) {
    mBuilder_ = nvUniquePtr<NvBuilder>(nvinfer1::createInferBuilder(mLogger_));
    if (mBuilder_ == nullptr) {
      INFER_ERROR << "Fail to create infer builder!";
      exit(kFAILURE);
    }
    std::string save_path = "/tmp";
    if (!mOptions_.save_path.empty()) {
      if (dirExists(mOptions_.save_path)) {
        save_path = mOptions_.save_path;
      } else {
        INFER_DEBUG
            << "Save engine path not exist! Engine will be saved in /tmp";
      }
    }
    std::string engine_cache_name{genEngineCacheName(save_path, model_)};

    if (!engine_file_.empty()) {
      engine_cache_name = save_path + "/" + engine_file_;
    }
    INFER_DEBUG << "Trying to save engine to " << save_path;
    createEngine(model_, engine_cache_name);
  }
}

std::string TrtInferImpl::genEngineCacheName(const std::string& save_path,
                                             const std::string& onnx_model) {
  std::string tmp_infos;

  tmp_infos += "~";
  tmp_infos += onnx_model;
  tmp_infos += "~";
  tmp_infos += std::to_string(CUDART_VERSION);
  tmp_infos += "~";
  tmp_infos += std::to_string(NV_TENSORRT_VERSION);
  tmp_infos += "~";
  tmp_infos += std::to_string(mOptions_.height);
  tmp_infos += "~";
  tmp_infos += std::to_string(mOptions_.width);
  tmp_infos += "~";
  tmp_infos += std::to_string(mOptions_.enable_fp16);
  tmp_infos += "~";
  tmp_infos += std::to_string(mOptions_.enable_int8);
  tmp_infos += "~";
  tmp_infos += std::to_string(mOptions_.enable_dla);

  // generate md5 code
  std::string md5_code{save_path + "/." + md5(tmp_infos) + "~"};

  return md5_code;
}

std::string TrtInferImpl::decryptModelFile(const std::string& model_file) {
  std::string model_buffer;
  if (pathExists(model_file)) {
    std::vector<char> extra_model_buffer;
    if (CryptorParser().readFileToBuffer(model_file, extra_model_buffer)) {
      std::string decrypt_model = CryptorParser().decrypt(extra_model_buffer);
      CryptorParser().parse(decrypt_model, mModelInfo_, model_buffer);
    }
    if (mModelInfo_.empty()) {
      model_buffer = model_file;
      model_format_ = ModelFormat::kONNX;
    }
  } else {
    INFER_ERROR << model_file
                << " do not exist, please check the path of the model file";
    exit(kFAILURE);
  }
  return model_buffer;
}

void TrtInferImpl::createEngine(const std::string& onnx_model,
                                const std::string& engine_file) {
  int8_t status{kSUCCESS};
  status = deserializeEngine(engine_file);

  if (status == kSUCCESS) {
    INFER_DEBUG << "Succeed in deserializing engine from disk, try to set up "
                   "inference engine...";
  } else {
    INFER_DEBUG
        << "Fail to deserialize engine from disk, try to build a new one.";

    status = buildEngineWithOnnx(onnx_model, engine_file);

    if (status == kSUCCESS) {
      INFER_DEBUG << "Succeed in buiding engine with onnx file, try to set up "
                     "inference engine...";
    } else {
      INFER_ERROR << "Fail to build engine with onnx file!";
      exit(kFAILURE);
    }
  }
  status = setUpInference();

  if (status == kFAILURE) {
    INFER_ERROR << "Fail to set up engine!";
    exit(kFAILURE);
  }
  INFER_DEBUG << "Succeed in setting up inference engine.";
}

int8_t TrtInferImpl::deserializeEngine(const std::string& engine_file) {
  if (pathExists(engine_file)) {
    std::vector<char> model_buffer;
    if (CryptorParser().readFileToBuffer(engine_file, model_buffer)) {
      std::string decrypt_cache;
      if (mOptions_.encrypt) {
        decrypt_cache = CryptorParser().decrypt(model_buffer);
      } else {
        decrypt_cache = std::string(model_buffer.begin(), model_buffer.end());
      }

      runtime =
          nvUniquePtr<NvRuntime>(nvinfer1::createInferRuntime(mLogger_));
      if (runtime == nullptr) {
        INFER_ERROR << "Fail to create infer runtime!";
        exit(kFAILURE);
      }
      if (mOptions_.enable_dla) {
        if (runtime->getNbDLACores() < 1) {
          INFER_DEBUG << "The platform have no dla core";
        } else {
          runtime->setDLACore(mOptions_.dla_core);
          INFER_DEBUG << "Set DLA core: " << runtime->getDLACore();
        }
      }
      mEngine_ = std::shared_ptr<NvEngine>(runtime->deserializeCudaEngine(
          decrypt_cache.c_str(), decrypt_cache.size()));
      if (mEngine_ == nullptr) {
        INFER_ERROR << "Fail to deserialize the cuda engine!";
        return kFAILURE;
      }

    } else {
      INFER_ERROR << "Fail to read existing engine cache file!";
      return kFAILURE;
    }
    return kSUCCESS;
  } else {
    INFER_ERROR << "engine file not exists!";
    return kFAILURE;
  }
}

int8_t TrtInferImpl::buildEngineWithOnnx(const std::string& onnx_model,
                                         const std::string& engine_file) {
  int8_t status{kSUCCESS};
  if (mBuilder_ == nullptr) {
    INFER_ERROR << "Builder is a nullptr!";
    return kFAILURE;
  }

  mFlags_ = 1U << static_cast<uint32_t>(NvNetDefCrtFlag::kEXPLICIT_BATCH);
  mNetwork_ = nvUniquePtr<NvNetDef>(mBuilder_->createNetworkV2(mFlags_));

  if (mNetwork_ == nullptr) {
    INFER_ERROR << "Fail to create network!";
    return kFAILURE;
  }

  auto parser = nvUniquePtr<nvonnxparser::IParser>(
      nvonnxparser::createParser(*mNetwork_, mLogger_));
  if (parser == nullptr) {
    INFER_ERROR << "Fail to create parser";
    return kFAILURE;
  }

  switch (model_format_) {
    case ModelFormat::kONNX: {
      if (!parser->parseFromFile(
              onnx_model.c_str(),
              static_cast<int>(nvinfer1::ILogger::Severity::kERROR))) {
        INFER_ERROR
            << "Fail to parse onnx model file, please check the trt support op";
        return kFAILURE;
      }
      break;
    }

    case ModelFormat::kEncryptONNX: {
      if (!parser->parse(onnx_model.c_str(), onnx_model.size())) {
        INFER_ERROR << "Fail to parse the encrypt onnx model file, please "
                       "check the trt support op";
        return kFAILURE;
      }
      break;
    }

    case ModelFormat::kUnknown: {
      INFER_ERROR << "Unknown model format, check your model!";
      return kFAILURE;
      break;
    }
    default: {
      INFER_ERROR << "Unsupport model format, check your model!";
      return kFAILURE;
    }
  }

  status = buildEngine();
  if (status == kFAILURE) {
    INFER_ERROR << "Fail to build an inference engine!";
    return kFAILURE;
  }

  INFER_DEBUG
      << "Finish building a new engine, trying to save engine to disk...";
  status = saveEngine(engine_file);
  if (status == kFAILURE) {
    INFER_ERROR << "Fail to save engine to disk!";
    return kFAILURE;
  }
  INFER_DEBUG << "Succeed in saving engine to disk.";

  return kSUCCESS;
}

int8_t TrtInferImpl::buildEngine() {
  int8_t status{kSUCCESS};
  auto m_config = nvUniquePtr<NvBuilderCfg>(mBuilder_->createBuilderConfig());
  if (m_config == nullptr) {
    INFER_ERROR << "Fail to create builder config!";
    return kFAILURE;
  }

  bool enable_int8{mOptions_.enable_int8};
  bool enable_fp16{mOptions_.enable_fp16};
  bool enable_dla{mOptions_.enable_dla};

  if (mNetwork_ != nullptr) {
    auto num_input = mNetwork_->getNbInputs();
    //         for (int32_t i = 0; i < num_input; ++i) {
    //             nvinfer1::ITensor *input_tensor = mNetwork_->getInput(i);
    //             nvinfer1::Dims dims = input_tensor->getDimensions();

    //             nvinfer1::Dims new_dims(dims);
    //             if (mOptions_.height > 0 && mOptions_.width > 0) {
    // // 而对于TensorRT8，需要将HW设为-1指示为大小可变

    // #if defined(USE_TENSORRT_V7) || defined(USE_TENSORRT_V8)
    //                 new_dims = nvinfer1::Dims4(dims.d[0], dims.d[1],
    //                 mOptions_.height, mOptions_.width);
    // #endif
    // #if defined(USE_TENSORRT_V8)
    //                 new_dims = nvinfer1::Dims4(dims.d[0], dims.d[1], -1, -1);
    // #endif
    //             }
    //             input_tensor->setDimensions(new_dims);
    //         }
  }

  // nvinfer1::Dims4 cali_dims = nvinfer1::Dims4(mOptions_.calibrate_batch, 8,
  // mOptions_.height, mOptions_.width);
  // std::unique_ptr<nvinfer1::IInt8Calibrator> calibrator;
  // profile
  // mProfiler_ = mBuilder_->createOptimizationProfile();
  // // TODO
  // if (mProfiler_) {
  //     auto num_input = mNetwork_->getNbInputs();
  //     for (int32_t i = 0; i < num_input; ++i) {
  //         nvinfer1::ITensor *input_tensor = mNetwork_->getInput(i);
  //         nvinfer1::Dims dims = input_tensor->getDimensions();
  //         auto const isDynamicInput = std::any_of(dims.d, dims.d +
  //         dims.nbDims, [](int32_t dim) { return dim == -1; });
  //         //TODO multi input
  //         nvinfer1::Dims min_dims = nvinfer1::Dims4(1, dims.d[1],
  //         mOptions_.height, mOptions_.width); nvinfer1::Dims opt_dims =
  //         nvinfer1::Dims4(1, dims.d[1], mOptions_.height, mOptions_.width);
  //         nvinfer1::Dims max_dims = nvinfer1::Dims4(1, dims.d[1],
  //         mOptions_.height, mOptions_.width); if
  //         (!mProfiler_->setDimensions(input_tensor->getName(),
  //         nvinfer1::OptProfileSelector::kMIN, min_dims)) {
  //             INFER_ERROR << "Set min dims profiler: inconsistency was
  //             detected!";
  //         }
  //         if (!mProfiler_->setDimensions(input_tensor->getName(),
  //         nvinfer1::OptProfileSelector::kOPT, opt_dims)) {
  //             INFER_ERROR << "Set opt dims profiler: inconsistency was
  //             detected!";
  //         }
  //         if (!mProfiler_->setDimensions(input_tensor->getName(),
  //         nvinfer1::OptProfileSelector::kMAX, max_dims)) {
  //             INFER_ERROR << "Set max dims profiler: inconsistency was
  //             detected!";
  //         }
  //     }
  //     if (mProfiler_->isValid()) {
  //         if (m_config->addOptimizationProfile(mProfiler_) == -1) {
  //             INFER_ERROR << "Input is no valid!";
  //         }
  //     } else {
  //         INFER_DEBUG << "Invalid optimization profile! Please check the
  //         dimensions of the network!";
  //     }
  // }

  m_config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, size_t(1) << mOptions_.max_workspace);

  if (enable_fp16) {
    INFER_DEBUG << "Set FP16 mode";
    if (!mBuilder_->platformHasFastFp16()) {
      INFER_DEBUG << "The platform do not support fp16";
    }
    m_config->setFlag(nvinfer1::BuilderFlag::kFP16);
  }

  if (enable_dla) {
    INFER_DEBUG << "Set DLA core  " << mOptions_.dla_core;
    if (mBuilder_->getNbDLACores() < 1) {
      INFER_DEBUG << "The platform have no dla core";
    } else {
      m_config->setDefaultDeviceType(nvinfer1::DeviceType::kDLA);
      m_config->setDLACore(mOptions_.dla_core);
      m_config->setFlag(nvinfer1::BuilderFlag::kGPU_FALLBACK);
    }
  }
  mEngineData_ = std::shared_ptr<NvHostMemory>(
      mBuilder_->buildSerializedNetwork(*mNetwork_, *m_config));
  if (mEngineData_ == nullptr) {
    INFER_ERROR << "Fail to build engine data with config!";
    return kFAILURE;
  }
  auto runtime =
          nvUniquePtr<NvRuntime>(nvinfer1::createInferRuntime(mLogger_));
  if (runtime == nullptr) {
    INFER_ERROR << "Fail to create infer runtime!";
    exit(kFAILURE);
  }
  mEngine_ = std::shared_ptr<NvEngine>(runtime->deserializeCudaEngine(mEngineData_->data(), mEngineData_->size()));

  if (mEngine_ == nullptr) {
    INFER_ERROR << "Fail to build engine with config!";
    return kFAILURE;
  }
  return kSUCCESS;
}

int8_t TrtInferImpl::saveEngine(const std::string& file_name) {
  if (file_name == "") {
    INFER_DEBUG << "Empty engine file name to save, skip save";
    return kSUCCESS;
  }
  if (mEngineData_ != nullptr) {
    std::ofstream engine_fstream(file_name, std::ofstream::binary);
    if (mOptions_.encrypt) {
      std::vector<char> engine_data_char(mEngineData_->size());
      const char* engine_data_ptr =
          reinterpret_cast<const char*>(mEngineData_->data());
      memcpy(engine_data_char.data(), engine_data_ptr, mEngineData_->size());
      std::string encrypt_cache = CryptorParser().encrypt(engine_data_char);
      engine_fstream.write(encrypt_cache.c_str(), encrypt_cache.size());
    } else {
      engine_fstream.write(static_cast<char*>(mEngineData_->data()),
                           mEngineData_->size());
    }
    engine_fstream.close();
    return kSUCCESS;
  } else {
    INFER_ERROR << "Engine is empty! Fail to save engine!";
    return kFAILURE;
  }
}

int8_t TrtInferImpl::setUpBindings() {
  mBindings_ = Bindings(mOptions_.use_managed, mOptions_.use_Unified_Address,
                        mOptions_.use_gpu_input, stream_);

  int32_t const num_bindings = mEngine_->getNbIOTensors();
  for (int32_t b = 0; b < num_bindings; ++b) {
    auto const& name = mEngine_->getIOTensorName(b);
    auto const& mode = mEngine_->getTensorIOMode(name);
    if (mode == nvinfer1::TensorIOMode::kINPUT) {
      mNumInput_++;
      nvinfer1::Dims const dims = mContext_->getTensorShape(name);
      bool const hasRuntimeDim = std::any_of(
          dims.d, dims.d + dims.nbDims, [](int32_t dim) { return dim == -1; });
      if (hasRuntimeDim) {
        nvinfer1::Dims rt_dims;
        rt_dims.nbDims = dims.nbDims;
        // rt_dims.MAX_DIMS = dims.nbDims;
        for (int i = 0; i < rt_dims.nbDims; ++i) {
          rt_dims.d[i] = dims.d[i] >= 0 ? dims.d[i] : mOptions_.batch_size;
        }
        for (int i = rt_dims.nbDims; i < rt_dims.MAX_DIMS; ++i) {
          rt_dims.d[i] = 0;
        }
        if (!mContext_->setInputShape(name, rt_dims)) {
          INFER_ERROR << "Fail to set binding dimensions!";
          return kFAILURE;
        }
      }
    }
    TensorInfo tensorInfo;
    tensorInfo.bindingIndex = b;
    getTensorInfo(tensorInfo);
    tensorInfo.updateVolume();
    mBindings_.addBinding(tensorInfo);
    mTensorInfos_.emplace_back(tensorInfo);
  }

  mBindings_.setTensorAddresses(*mContext_);
  return kSUCCESS;
}

int8_t TrtInferImpl::setUpInference() {
  if (mEngine_ == nullptr) {
    INFER_ERROR << "Fail to build inference engine!";
    return kFAILURE;
  }
  mContext_ = nvUniquePtr<NvExeCtx>(mEngine_->createExecutionContext());
  if (mContext_ == nullptr) {
    INFER_ERROR << "Fail to create execution context";
    return kFAILURE;
  }

  if (mOptions_.profiler) {
    mProfiler = new Profiler;
    mContext_->setProfiler(mProfiler);
    mContext_->setEnqueueEmitsProfile(false);
  }

  if (kFAILURE == setUpBindings()) {
    INFER_ERROR << "failed in set up bindings";
    return kFAILURE;
  }
  // for use cuda graph
  mContext_->enqueueV3(stream_);
  BASE_CUDA_CHECK(cudaStreamSynchronize(stream_));
  return kSUCCESS;
}

void TrtInferImpl::forward() {
#ifndef CMAKE_DEV
  nvtxEventAttributes_t eventAttrib = {
    .version = NVTX_VERSION,
    .size = NVTX_EVENT_ATTRIB_STRUCT_SIZE,
    .category = 0,
    .colorType = NVTX_COLOR_ARGB,
    .color = 0x00ffffff,
    .payloadType = NVTX_PAYLOAD_UNKNOWN,
    .reserved0 = 0,
    .payload{.fValue = 0},
    .messageType = NVTX_MESSAGE_TYPE_ASCII,
    .message{.ascii = model_file_name_.c_str()},
  };
  nvtxRangePushEx(&eventAttrib);
#endif
  transferInputToDevice();
  enqueue();
  transferOutputToHost();
  // BASE_CUDA_CHECK(cudaStreamSynchronize(stream_));
  cudaSteamSync();
#ifndef CMAKE_DEV
  nvtxRangePop();
#endif
}

void TrtInferImpl::transferInputToDevice() {
  mBindings_.transferInputToDevice();
}

void TrtInferImpl::enqueue() {
  if (mContext_ != nullptr) {
    if (mOptions_.use_cuda_graph && graphFlag_ == GraphFlag::kFIRST_FRAME) {
      BASE_CUDA_CHECK(cudaStreamBeginCapture(stream_,
                                             cudaStreamCaptureModeThreadLocal));
      mContext_->enqueueV3(stream_);
      auto ret = cudaStreamEndCapture(stream_, &mGraph_);
      graphFlag_ = GraphFlag::kFAILED;
      if (ret == cudaSuccess) {
#if CUDA_VERSION >= 12000
        cudaGraphInstantiate(&mGraphExec_, mGraph_,  0);
#else
        cudaGraphInstantiate(&mGraphExec_, mGraph_, NULL, NULL, 0);
#endif
        BASE_CUDA_CHECK(cudaGraphDestroy(mGraph_));
        graphFlag_ = GraphFlag::kSUCCESS;
      } else if (mGraph_ != nullptr) {
        assert(ret == cudaSuccess);
        mGraph_ = nullptr;
      }
    }

    if (graphFlag_ == GraphFlag::kSUCCESS) {  // usecuda graph
      BASE_CUDA_CHECK(cudaGraphLaunch(mGraphExec_, stream_));
      INFER_TRACE << "using cudaGraphLaunch!";
    } else {
      INFER_TRACE << "using enqueueV3!";
      if (!mContext_->enqueueV3(stream_)) {
        INFER_ERROR << "Fail to perform enqueueV3!";
        exit(kFAILURE);
      }
    }
  } else {
    INFER_ERROR << "The infer engine have not been initialized, Please init "
                   "the infer engine first!";
    exit(kFAILURE);
  }
  if (mContext_->getProfiler() && !mContext_->getEnqueueEmitsProfile() &&
      !mContext_->reportToProfiler()) {
    INFER_ERROR
        << "Failed to collect layer timing info from previous enqueue()";
  }
}

void TrtInferImpl::transferOutputToHost() { mBindings_.transferOutputToHost(); }

void TrtInferImpl::cudaSteamSync() {
  uint32_t count = 0;
  while (true) {
    count++;
    cudaError_t ret = cudaStreamQuery(stream_);
    if (cudaSuccess == ret) {
      break;
    } else if (cudaErrorNotReady == ret) {
      // 2ms
      if (count > 500) {
        count = 0;
        INFER_ERROR << "DEBUG, sleep 500, ERROR:" << ret;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    } else if (cudaErrorInvalidResourceHandle == ret) {
      // 2ms
      if (count > 500) {
        count = 0;
        INFER_ERROR << "DEBUG, sleep 500, ERROR:" << ret;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    } else {
      // 2ms
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      INFER_ERROR << "DEBUG ERROR:" << ret;
      continue;
    }
  }
}

void TrtInferImpl::forward(const std::vector<void*>& inputs) {
  for (size_t i = 0; i < inputs.size(); ++i) {
    auto const name = mEngine_->getIOTensorName(i);
    mContext_->setTensorAddress(name, inputs[i]);
  }
  forward();
}

void TrtInferImpl::getTensorInfo(TensorInfo& tensorInfo) {
  auto const b = tensorInfo.bindingIndex;
  auto const name = mEngine_->getIOTensorName(b);
  tensorInfo.name = name;

  // Use enqueueV3.
  tensorInfo.dims = mContext_->getTensorShape(name);
  tensorInfo.isDynamic =
      std::any_of(tensorInfo.dims.d, tensorInfo.dims.d + tensorInfo.dims.nbDims,
                  [](int32_t dim) { return dim == -1; });
  tensorInfo.comps = mEngine_->getTensorComponentsPerElement(name);
  // tensorInfo.strides = mContext_->getTensorStrides(name);
  tensorInfo.vectorDimIndex = mEngine_->getTensorVectorizedDim(name);
  tensorInfo.isInput =
      mEngine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT;
  tensorInfo.dataType = mEngine_->getTensorDataType(name);
}

}  // namespace inference
}  // namespace robosense
