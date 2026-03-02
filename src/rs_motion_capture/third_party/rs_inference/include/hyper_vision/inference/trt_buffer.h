#ifndef HYPER_VISION_INFERENCE_TRT_BUFFER_H
#define HYPER_VISION_INFERENCE_TRT_BUFFER_H
#include <cuda_runtime_api.h>
#include <cuda.h>

#include "NvInfer.h"
#include "hyper_vision/common/log.h"
#include "hyper_vision/inference/infer_buffer.h"
#include "hyper_vision/inference/trt_utils.h"

namespace robosense {
namespace inference {

inline void* safeCudaMalloc(size_t memSize, cudaStream_t stream) {
  void* deviceMem;
#if CUDA_VERSION >= 11040
  BASE_CUDA_CHECK(cudaMallocAsync(&deviceMem, memSize, stream));
#else
  BASE_CUDA_CHECK(cudaMalloc(&deviceMem, memSize));
#endif
  if (deviceMem == nullptr) {
    INFER_ERROR << "CUDA Malloc ERROR!";
    exit(kFAILURE);
  }
  // BASE_CUDA_CHECK(cudaMemset(deviceMem, 0, memSize));
  return deviceMem;
}

inline void safeCudaFree(void* device_ptr, cudaStream_t stream) {
#if CUDA_VERSION >= 11040
  BASE_CUDA_CHECK(cudaFreeAsync(device_ptr, stream));
#else
  BASE_CUDA_CHECK(cudaFree(device_ptr));
#endif
}

inline void* safeCudaMallocHost(size_t memSize) {
  void* hostMem;
  BASE_CUDA_CHECK(cudaMallocHost(&hostMem, memSize));
  if (hostMem == nullptr) {
    INFER_ERROR << "CUDA MallocHost ERROR!";
    exit(kFAILURE);
  }
  // BASE_CUDA_CHECK(cudaMemset(hostMem, 0, memSize));
  return hostMem;
}

inline void* safeCudaMallocManaged(size_t memSize) {
  void* managedMem;
  BASE_CUDA_CHECK(cudaMallocManaged(&managedMem, memSize));
  if (managedMem == nullptr) {
    INFER_ERROR << "CUDA MallocManaged ERROR!";
    exit(kFAILURE);
  }
  // BASE_CUDA_CHECK(cudaMemset(managedMem, 0, memSize));
  return managedMem;
}

inline void safeCudaFreeHost(void* host_ptr) {
  BASE_CUDA_CHECK(cudaFreeHost(host_ptr));
}

struct DeviceAllocator {
  void* operator()(size_t size, cudaStream_t stream) { return safeCudaMalloc(size, stream); }
};

struct DeviceDeallocator {
  void operator()(void* ptr, cudaStream_t stream) { safeCudaFree(ptr, stream); }
};

struct HostAllocator {
  void* operator()(size_t size, cudaStream_t stream) { return safeCudaMallocHost(size); }
};

struct HostDeallocator {
  void operator()(void* ptr, cudaStream_t stream) { safeCudaFreeHost(ptr); }
};

struct ManagedAllocator {
  void* operator()(size_t size, cudaStream_t stream) { return safeCudaMallocManaged(size); }
};

struct HostUnifiedAddressAllocator {
  void* operator()(size_t size, cudaStream_t stream) {
    void* hostMem;
    BASE_CUDA_CHECK(
        cudaHostAlloc(&hostMem, size,
                      cudaHostAllocDefault));  // try cudaHostAllocWriteCombined
    if (hostMem == nullptr) {
      INFER_ERROR << "CUDA cudaHostAlloc ERROR!";
      exit(kFAILURE);
    }
    // BASE_CUDA_CHECK(cudaMemset(hostMem, 0, size));
    return hostMem;
  }
};

struct HostUnifiedAddressDeallocator {
  void operator()(void* ptr, cudaStream_t stream) { safeCudaFreeHost(ptr); }
};

using TrtDeviceBuffer = InferBuffer<DeviceAllocator, DeviceDeallocator>;
using TrtHostBuffer = InferBuffer<HostAllocator, HostDeallocator>;
using TrtManagedBuffer = InferBuffer<ManagedAllocator, DeviceDeallocator>;
using TrtUnifiedAddressBuffer =
    InferBuffer<HostUnifiedAddressAllocator, HostUnifiedAddressDeallocator>;

//!
//! \class MirroredBuffer
//! \brief Coupled host and device buffers
//!
class IMirroredBuffer {
 public:
  //!
  //! Allocate memory for the mirrored buffer give the size
  //! of the allocation.
  //!
  virtual void allocate(size_t size) = 0;

  //!
  //! Get the pointer to the device side buffer.
  //!
  //! \return pointer to device memory or nullptr if uninitialized.
  //!
  virtual void* getDeviceBuffer() const = 0;

  //!
  //! Get the pointer to the host side buffer.
  //!
  //! \return pointer to host memory or nullptr if uninitialized.
  //!
  virtual void* getHostBuffer() const = 0;

  virtual void setDeviceBuffer(void* device_buffer) = 0;
  virtual void setHostBuffer(void* host_buffer) = 0;

  //!
  //! Copy the memory from host to device.
  //!
  virtual void hostToDevice() = 0;

  //!
  //! Copy the memory from device to host.
  //!
  virtual void deviceToHost() = 0;

  //!
  //! Interface to get the size of the memory
  //!
  //! \return the size of memory allocated.
  //!
  virtual size_t getSize() const = 0;

  //!
  //! Virtual destructor declaraion
  //!
  virtual ~IMirroredBuffer() = default;

  // delete default constructor
  IMirroredBuffer() = delete;

  explicit IMirroredBuffer(cudaStream_t stream) : stream_(stream) {}

  // stream from constructor
  cudaStream_t stream_;

};  // class IMirroredBuffer

//!
//! Class to have a separate memory buffer for discrete device and host
//! allocations.
//!
class DiscreteMirroredBuffer : public IMirroredBuffer {
 public:
  explicit DiscreteMirroredBuffer(cudaStream_t stream) : IMirroredBuffer(stream) {}
  void allocate(size_t size) {
    mSize = size;
    mHostBuffer.allocate(size);
    mDeviceBuffer.set_stream(stream_);
    mDeviceBuffer.allocate(size);
  }

  void* getDeviceBuffer() const { return mDeviceBuffer.get(); }

  void* getHostBuffer() const { return mHostBuffer.get(); }

  void setDeviceBuffer(void* device_buffer) {
    mDeviceBuffer.set(device_buffer);
  }
  void setHostBuffer(void* host_buffer) { mHostBuffer.set(host_buffer); }

  void hostToDevice() {
    BASE_CUDA_CHECK(cudaMemcpyAsync(mDeviceBuffer.get(), mHostBuffer.get(),
                                    mSize, cudaMemcpyHostToDevice, stream_));
  }

  void deviceToHost() {
    BASE_CUDA_CHECK(cudaMemcpyAsync(mHostBuffer.get(), mDeviceBuffer.get(),
                                    mSize, cudaMemcpyDeviceToHost, stream_));
  }

  size_t getSize() const { return mSize; }

 private:
  size_t mSize{0};
  TrtHostBuffer mHostBuffer;
  TrtDeviceBuffer mDeviceBuffer;
};  // class DiscreteMirroredBuffer

//!
//! Class to have a GPU device memory buffer.
//!
class GPUDeviceBuffer : public IMirroredBuffer {
 public:
  explicit GPUDeviceBuffer(cudaStream_t stream) : IMirroredBuffer(stream) {}
  void allocate(size_t size) {
    mSize = size;
    mHostBuffer.allocate(size);
    mDeviceBuffer.set_stream(stream_);
    mDeviceBuffer.allocate(size);
  }

  void* getDeviceBuffer() const { return mDeviceBuffer.get(); }

  void* getHostBuffer() const { return mHostBuffer.get(); }

  void setDeviceBuffer(void* device_buffer) {
    mDeviceBuffer.set(device_buffer);
  }

  void setHostBuffer(void* host_buffer) { mHostBuffer.set(host_buffer); }
  void hostToDevice() {
    // Dose nothing
  }

  void deviceToHost() {
    BASE_CUDA_CHECK(cudaMemcpyAsync(mHostBuffer.get(), mDeviceBuffer.get(),
                                    mSize, cudaMemcpyDeviceToHost, stream_));
  }

  size_t getSize() const { return mSize; }

 private:
  size_t mSize{0};
  TrtDeviceBuffer mDeviceBuffer;
  TrtHostBuffer mHostBuffer;
};

//!
//! Class to have a unified memory buffer for embedded devices.
//!
class UnifiedMirroredBuffer : public IMirroredBuffer {
 public:
  explicit UnifiedMirroredBuffer(cudaStream_t stream) : IMirroredBuffer(stream) {}
  void allocate(size_t size) {
    mSize = size;
    mBuffer.allocate(size);
  }

  void* getDeviceBuffer() const { return mBuffer.get(); }

  void* getHostBuffer() const { return mBuffer.get(); }

  void setDeviceBuffer(void* device_buffer) { mBuffer.set(device_buffer); }

  void setHostBuffer(void* host_buffer) { mBuffer.set(host_buffer); }

  void hostToDevice() {
    // Does nothing since we are using unified memory.
  }

  void deviceToHost() {
    // Does nothing since we are using unified memory.
  }

  size_t getSize() const { return mSize; }

 private:
  size_t mSize{0};
  TrtManagedBuffer mBuffer;
};

//!
//! Class for jetson which has a unified addressing(physically unified memory).
//!
class UnifiedAddressBuffer : public IMirroredBuffer {
 public:
  explicit UnifiedAddressBuffer(cudaStream_t stream) : IMirroredBuffer(stream) {}
  void allocate(size_t size) {
    mSize = size;
    mBuffer.allocate(size);

    unsigned int flags;
    cudaError_t ret = cudaGetDeviceFlags(&flags);
    if (cudaSuccess != ret || cudaDeviceMapHost != (flags & 0xf)) {
      INFER_ERROR
          << "the CUDA context is not support the cudaDeviceMapHost flag "
          << flags << ", " << ret;
      exit(kFAILURE);
    }
    void* deviceMem;
    ret = cudaHostGetDevicePointer(&deviceMem, getHostBuffer(), 0);
    if (cudaSuccess != ret) {
      INFER_ERROR << "the CUDA memory map failed! " << ret;
      exit(kFAILURE);
    }
    TrtHostMapToDevice = deviceMem;
  }

  void* getDeviceBuffer() const { return TrtHostMapToDevice; }

  void* getHostBuffer() const { return mBuffer.get(); }

  void setDeviceBuffer(void* device_buffer) {
    TrtHostMapToDevice =
        device_buffer;  // device buffer must map to host buffer
  }
  void setHostBuffer(void* host_buffer) { mBuffer.set(host_buffer); }

  void hostToDevice() {
    // printf("hostToDevice:%p, %p\n", TrtHostMapToDevice, mBuffer.get());
    // Does nothing since we are using unified memory.
  }

  void deviceToHost() {
    // printf("deviceToHost:%p, %p\n", mBuffer.get(), TrtHostMapToDevice);
    // Does nothing since we are using unified memory.
  }

  size_t getSize() const { return mSize; }

 private:
  size_t mSize{0};
  TrtUnifiedAddressBuffer mBuffer;
  void* TrtHostMapToDevice;
};

}  // namespace inference
}  // namespace robosense
#endif