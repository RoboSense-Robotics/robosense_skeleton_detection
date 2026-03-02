#ifndef HYPER_VISION_INFERENCE_TRT_BINDINGS_H
#define HYPER_VISION_INFERENCE_TRT_BINDINGS_H

#include <functional>
#include <unordered_map>

#include <cuda_runtime_api.h>

#include "NvInfer.h"
// #include "cuda.h"
// #include "cuda_fp16.h"
#include "hyper_vision/common/log.h"
#include "hyper_vision/inference/infer_buffer.h"
#include "hyper_vision/inference/trt_buffer.h"
namespace robosense {
namespace inference {
class OutputAllocator : public nvinfer1::IOutputAllocator {
 public:
  OutputAllocator(IMirroredBuffer* buffer) : mBuffer(buffer) {}

  void* reallocateOutput(char const* tensorName, void* currentMemory,
                         uint64_t size, uint64_t alignment) noexcept override {
    // Some memory allocators return nullptr when allocating zero bytes, but
    // TensorRT requires a non-null ptr even for empty tensors, so allocate a
    // dummy byte.
    size = std::max(size, static_cast<uint64_t>(1));
    if (size > mSize) {
      mBuffer->allocate(roundUp(size, alignment));
      mSize = size;
    }
    return mBuffer->getDeviceBuffer();
  }

  void notifyShape(char const* tensorName,
                   nvinfer1::Dims const& dims) noexcept override {}

  IMirroredBuffer* getBuffer() { return mBuffer.get(); }

  void* getDeviceBuffer() { return mBuffer->getDeviceBuffer(); }
  void* getHostBuffer() { return mBuffer->getHostBuffer(); }
  void setDeviceBuffer(void* device_buffer) {
    mBuffer->setDeviceBuffer(device_buffer);
  }
  void setHostBuffer(void* host_buffer) { mBuffer->setHostBuffer(host_buffer); }
  virtual ~OutputAllocator() {}

 private:
  std::unique_ptr<IMirroredBuffer> mBuffer;
  uint64_t mSize{};
};

struct Binding {
  std::string name;
  bool isInput{false};
  std::unique_ptr<IMirroredBuffer> buffer;
  std::unique_ptr<OutputAllocator> outputAllocator;
  int32_t volume{0};
  nvinfer1::DataType dataType{nvinfer1::DataType::kFLOAT};
};

class Bindings {
 public:
  Bindings() {}
  explicit Bindings(bool useManaged) : mUseManaged(useManaged) {}
  Bindings(bool useManaged, bool useUnifiedAddress, bool useGPUInput, cudaStream_t stream)
      : mUseManaged(useManaged),
        mUseUnifiedAddress(useUnifiedAddress),
        mUseGPUInput(useGPUInput),
        mStream(stream) {}

  void addBinding(TensorInfo const& tensorInfo);

  void** getDeviceBuffers();
  // void** getHostBuffers();

  void transferInputToDevice();

  void transferOutputToHost();

  const Binding& getBinding(const std::string& name) const;
  const Binding& getInputBinding(int32_t index) const;
  const Binding& getOutputBinding(int32_t index) const;

  std::unordered_map<std::string, int> getBindings(
      std::function<bool(Binding const&)> predicate) const;

  bool setTensorAddresses(nvinfer1::IExecutionContext& context) const;

  size_t size() const { return mBindings.size(); }
  void reset() {
    mBindings.clear();
    mDevicePointers.clear();
    // mHostPointers.clear();
    mNames.clear();
  }
  int32_t mNumInput_{0};

 private:
  std::unordered_map<std::string, int32_t> mNames;
  std::vector<Binding> mBindings;
  std::vector<void*> mDevicePointers;
  // std::vector<void*> mHostPointers;
  bool mUseManaged{false};
  bool mUseUnifiedAddress{false};
  bool mUseGPUInput{false};
  cudaStream_t mStream{0};
};
}  // namespace inference
}  // namespace robosense
#endif