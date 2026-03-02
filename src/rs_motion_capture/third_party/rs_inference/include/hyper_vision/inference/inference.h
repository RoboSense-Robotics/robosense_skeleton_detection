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
#ifndef HYPER_VISION_INFERENCE_INFERENCE_H
#define HYPER_VISION_INFERENCE_INFERENCE_H

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cuda_runtime_api.h>

#include "hyper_vision/common/register.h"

namespace robosense {
namespace inference {

constexpr int8_t kSUCCESS{0};
constexpr int8_t kFAILURE{-1};

///
/// @brief The deep model format use for inference. Support raw onnx model,
/// encrypt onnx model and engine
///        fine.
///
///
enum class ModelFormat : uint8_t { kONNX, kEncryptONNX, kENGINE, kUnknown };

///
/// @brief The device type whether to get the data of the network. Support kCPU,
/// kGPU and kDSP device,
///        kNPU
///
///
enum class DeviceType : uint8_t { kCPU, kGPU, kDSP, kNPU };

///
/// @brief Currently supported inference engine type. Support kTENSORRT, kTIDL
/// and kHBDNN,
///
///
enum class EngineType : uint8_t { kTENSORRT, kTIDL, kHBDNN };

///
/// @brief The debug level to print information on terminal
///
///
enum class DebugLevel : uint8_t { kTRACE, kDEBUG, kINFO, kERROR };

///
/// @brief Model file handler to store the path and the model format of the deep
/// model.
///
///
struct ModelFileHandler {
  /// @brief modelFormat the model format, the raw onnx model, encrypt onnx
  /// model or the engine model
  ModelFormat modelFormat;

  /// @brief string name where the model stored
  std::string onnxFile;

  /// @brief the tensorrt generated engine file
  std::string engineFile;

  /// @brief io description file used in tidl
  std::string ioDescFile;

  /// @brief net binary file used in tidl
  std::string netBinFile;

  /// @brief tidl encrypt model file
  std::string tidlEncryptModel;

  /// @brief hb model file use in HBDnn
  std::string hbModelFile;
};

///
/// @brief Dimensions type to store the shape of input and output dimensions
///
///
struct Dims {
  static const int32_t MAX_DIMS = 8;
  int32_t nbDims;
  int32_t d[MAX_DIMS];
};

///
/// @brief The option configs to build a inference engine
///
///
struct InferOptions {
  /// @brief workspace, only support in tensorrt
  size_t max_workspace{30};

  /// @brief infer batch size
  int32_t batch_size{1};

  /// @brief device to perform inference, only support in tensorrt
  uint8_t device_id{0};

  /// @brief network input height, use in building the engine, only support in
  /// resizable engine
  uint32_t height{0};

  /// @brief network input width, use in building the engine, only support in
  /// resizable engine
  uint32_t width{0};

  /// @brief to perform int8 inference, only support int8 model
  bool enable_int8{false};

  /// @brief set enable_int8 true, calibration data path must be provided to
  /// perform quantization calibration
  std::string calibrate_data_path;

  /// if set enable_int8 true, where to save the calibration table
  std::string calibrate_cache_path;

  /// @brief set enable_int8, the batch size for int8 calibration
  int32_t calibrate_batch{1};

  /// @brief enable_int8, this option is to determine whether keep the network
  /// output in float precision (fp16 or fp32)
  bool keep_output_float{true};

  /// @brief to perform fp16 inference, only support in fp16-supported device
  bool enable_fp16{false};

  /// @brief to enable dla (deep learning accelerator), only support in tensorrt
  /// with dla-supported device
  bool enable_dla{false};

  /// the dla core index, only support in tensorrt with dla-supported device
  int32_t dla_core{0};

  /// @brief where to save the engine file if use tensorrt
  std::string save_path;

  /// @brief set cuda stream
  // cudaStream_t infer_stream;
  /// @brief set cuda stream priority
  int infer_stream_priority{0};

  /// @brief reserved_1
  bool use_reserved_1;

  /// @brief reserved_2
  bool use_reserved_2;

  /// @brief reserved_3
  bool use_reserved_3;

  //
  ModelFormat model_format;

  //
  bool encrypt{false};
  bool use_managed{false};
  bool use_Unified_Address{false};
  bool use_gpu_input{false};
  bool use_cuda_graph{false};
  bool profiler{false};
};

///
/// @brief Abstract InferEngine class to define api of the inference engine
///
///
class InferEngine {
 public:
  /// @brief std::shared_ptr<InferEngine>
  using Ptr = std::shared_ptr<InferEngine>;

  ///
  /// @brief Read model file from file, before init the infer engine, this
  /// function must be
  ///        call first to read model file into buffer. The user should specify
  ///        the model file path and the model file format. e.g. The user use
  ///        raw onnx model file add the ModelFormat should be
  ///        ModelFormat::kONNX
  ///
  /// @param file_handler
  ///
  virtual void readModelFromFile(
      const ModelFileHandler& file_handler) noexcept = 0;

  ///
  /// @brief Initialize the inference engine with provied options. This function
  /// will (1)
  ///        Create infer engine (2) Allocate memory for input and output
  ///        according to the network input and output size. However, user can
  ///        set a different input size if the network support resizable input.
  ///        e.g. the network is a fully convolutional network.
  ///
  /// @param options
  ///
  virtual void init(const InferOptions& options) noexcept = 0;

  ///
  /// @brief Reset the model engine, free the allocating memory.
  ///
  ///
  virtual void reset() noexcept = 0;

  ///
  /// @brief Get the neural network model version, will be empty if no model
  /// version is
  ///        provided.
  ///
  /// @return std::string e.g."v1.1.3.M1.20210727".
  ///
  virtual std::string getModelVersion() const noexcept = 0;

  ///
  /// @brief Get the extra deep model's infomation if the model is a descrypt
  /// model.
  ///
  /// @return std::map<std::string, std::string> a map if the model is a decrypt
  /// model,
  ///         else empty if not.
  ///
  virtual std::map<std::string, std::string> getModelInfos() const noexcept = 0;

  ///
  /// @brief Get the input name of the network.
  ///
  /// @param input_index The input index of the network.
  /// @return std::string
  ///
  virtual std::string getInputName(int32_t input_index) const noexcept = 0;

  ///
  /// @brief Get the output name of the network.
  ///
  /// @param output_index The output index of the network.
  /// @return std::string
  ///
  virtual std::string getOutputName(int32_t output_index) const noexcept = 0;

  ///
  /// @brief Do inference on engine. Before calling this function to perform
  /// inference,
  ///        getInputPtr() must first call to get the data pointer of the
  ///        network input on CPU or GPU, depend on the device type. Please note
  ///        that the return pointer is already point to a block of allocated
  ///        memory, so it is safe to fill data with the return data pointer. If
  ///        you want to get the inference result, just call the getOutputPtr()
  ///        to get the pointer of the network output.
  ///
  virtual void forward() noexcept = 0;
  virtual void forward(const std::vector<void*>& input) noexcept = 0;

  virtual void transferInputToDevice() noexcept = 0;
  virtual void enqueue() noexcept = 0;
  virtual void transferOutputToHost() noexcept = 0;
  virtual void sync() noexcept = 0;

  ///
  /// @brief Do inference on engine, with external input and output. The device
  /// indicate
  ///        which device will hold the data. It is not recommenedly to use this
  ///        function to perform inference, use forward() instead.
  ///
  /// @param input
  /// @param output
  ///
  virtual void forward(const std::vector<void*>& input,
                       const std::vector<void*>& output) noexcept = 0;

  ///
  /// @brief Get the number of network inputs
  ///
  /// @return The number of network inputs
  ///
  virtual int32_t getNumInputs() const noexcept = 0;

  ///
  /// @brief Get the number of network outputs
  ///
  /// @return The number of network outputs
  ///
  virtual int32_t getNumOutputs() const noexcept = 0;

  ///
  /// @brief Get the stream of network
  ///
  /// @return The stream of network
  ///
  virtual cudaStream_t getStream() const noexcept = 0;

  ///
  /// @brief Set the stream of network
  ///
  /// @return The stream of network
  ///
  virtual void setStream(cudaStream_t stream) noexcept = 0;

  ///
  /// @brief Get the stream priority
  ///
  /// @return The priority of stream
  ///
  virtual int getStreamPriority() const noexcept = 0;
  ///
  /// @brief Get the input pointer object
  ///
  /// @param input_index The input index where to get the pointer. Note that
  /// @param device The device where to get the pointer, support device is
  ///               iCPU, iGPU, iDSP
  /// @return void* The pointer which pointed to the network input
  ///
  virtual void* getInputPtr(int32_t input_index,
                            DeviceType device) noexcept = 0;

  ///
  /// @brief Get the input pointer object
  ///
  /// @param name Access the input pointer through name. Same logic function
  ///             with getInputPtr(int input_index, DeviceType device).
  /// @param device
  /// @return void*
  ///
  virtual void* getInputPtr(const std::string& name,
                            DeviceType device) noexcept = 0;

  ///
  /// @brief Get the output pointer object
  ///
  /// @param output_index The output index where to get the pointer
  /// @param device The device where to get the pointer, support device is
  ///               iCPU, iGPU, iDSP
  /// @return void* The pointer which pointed to the network output
  ///
  virtual void* getOutputPtr(int32_t output_index,
                             DeviceType device) const noexcept = 0;

  ///
  /// @brief Get the output pointer object
  ///
  /// @param name Access the output pointer through name. Same logic function
  ///             with getOutputPtr(int output_index, DeviceType device).
  /// @param device
  /// @return void*
  ///
  virtual void* getOutputPtr(const std::string& name,
                             DeviceType device) const noexcept = 0;

  ///
  /// @brief Get the input size of the network,  in bytes
  ///
  /// @param input_index The input index of the network
  /// @return size_t
  ///
  virtual size_t getInputSize(int32_t input_index) const noexcept = 0;

  ///
  /// @brief Get the input size of the network,  in bytes
  ///
  /// @param name The input name of the network
  /// @return size_t
  ///
  virtual size_t getInputSize(const std::string& name) const noexcept = 0;

  ///
  /// @brief Get the output size of the network,  in bytes
  ///
  /// @param output_index The output index of the network
  /// @return size_t
  ///
  virtual size_t getOutputSize(int32_t output_index) const noexcept = 0;

  ///
  /// @brief Get the input size of the network,  in bytes
  ///
  /// @param name The output name of the network
  /// @return size_t
  ///
  virtual size_t getOutputSize(const std::string& name) const noexcept = 0;

  ///
  /// @brief Get the input dimension according to input index, in NCHW format
  ///
  /// @param input_index
  /// @return Dims
  ///
  virtual Dims getInputDims(int32_t input_index) const noexcept = 0;

  ///
  /// @brief Get the output dimension according to output index, in NCHW format
  ///
  /// @param output_index
  /// @return Dims
  ///
  virtual Dims getOutputDims(int32_t output_index) const noexcept = 0;

  ///
  /// @brief Get the output scale if the network has been quantized (8bits or
  /// 16bits)
  ///
  /// @param output_index
  /// @return float
  ///
  virtual float getOutputScale(int32_t output_index) const noexcept = 0;

  ///
  /// @brief Get the engine inference latency, in ms
  ///
  /// @return float
  ///
  virtual float getInferenceLatency() const noexcept = 0;
  virtual void getProfilerInfo() const noexcept = 0;
  virtual bool setTensorAddress(const std::string& name,
                                void* address) noexcept = 0;
  virtual void* getTensorAddress(const std::string& name) noexcept = 0;
  virtual void changeTensorAddress(const std::string& src_name,
                                   const std::string& dst_name) noexcept = 0;
  virtual void changeTensorAddress(const std::string& src_name,
                                   const std::string& dst_name,
                                   void* src_device_ptr,
                                   void* src_host_ptr) noexcept = 0;
  ///
  /// @brief Set the Debug Level
  ///
  /// @param level
  ///
  virtual void setDebugLevel(DebugLevel level) noexcept;

  virtual ~InferEngine() = default;
};

REGISTER_REGISTER(InferEngine);
#define RS_REGISTER_INFER_ENGINE(name) REGISTER_CLASS(InferEngine, name)

const std::map<EngineType, std::string> kInferEngine2NameMap{
    {EngineType::kTENSORRT, "TrtInfer"},
    {EngineType::kTIDL, "TidlInfer"},
    {EngineType::kHBDNN, "HbdnnInfer"}};

const std::map<std::string, EngineType> kName2InferEngineMap{
    {"TensorRT", EngineType::kTENSORRT},
    {"TIDL", EngineType::kTIDL},
    {"Hbdnn", EngineType::kHBDNN}};
///
/// @brief Create an Infer Engine object
///
/// @param engine
/// @return InferEngine::Ptr
///
InferEngine::Ptr createInferEngine(EngineType engine);

///
/// @brief Create an Infer Engine object
///
/// @param engine
/// @return InferEngine::Ptr
///
InferEngine::Ptr createInferEngine(const std::string& engine);

}  // namespace inference
}  // namespace robosense

#endif  // HYPER_VISION_INFERENCE_INFERENCE_H
