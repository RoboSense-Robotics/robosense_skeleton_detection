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

#ifndef HYPER_VISION_INFERENCE_INFER_BUFFER_H
#define HYPER_VISION_INFERENCE_INFER_BUFFER_H

#include <vector>

namespace robosense {
namespace inference {

#define MALLOC_ALIGN 32
#define MALLOC_OVERREAD 64

///
/// @brief A common memory allocator and deallocator
///
/// @tparam _Tp
/// @param ptr
/// @param n
/// @return _Tp*
///
template <typename _Tp>
static inline _Tp* alignPtr(_Tp* ptr, int n = static_cast<int>(sizeof(_Tp))) {
  return reinterpret_cast<_Tp*>(((size_t)ptr + n - 1) & -n);
}

///
/// @brief Allocate memory with alignment
///
/// @param size
/// @return void*
///
static inline void* fastMalloc(size_t size) {
  unsigned char* udata = (unsigned char*)malloc(size + sizeof(void*) +
                                                MALLOC_ALIGN + MALLOC_OVERREAD);
  if (!udata) {
    return 0;
  }
  unsigned char** adata = alignPtr((unsigned char**)udata + 1, MALLOC_ALIGN);
  adata[-1] = udata;
  return adata;
}

///
/// @brief Free the memory allocated by fastMalloc
///
/// @param ptr
///
static inline void fastFree(void* ptr) {
  unsigned char* udata = ((unsigned char**)ptr)[-1];
  free(udata);
}

/// @brief This InferBuffer template store the data for network's input and
/// output
///
/// @tparam AllocatorType
/// @tparam DeallocatorType
///
template <typename AllocatorType, typename DeallocatorType>
class InferBuffer {
 public:
  InferBuffer() = default;

  InferBuffer(const InferBuffer&) = delete;

  InferBuffer& operator=(const InferBuffer&) = delete;

  InferBuffer(InferBuffer&& rhs) {
    reset(rhs.mPtr);
    rhs.mPtr = nullptr;
  }

  InferBuffer& operator=(InferBuffer&& rhs) {
    if (this != &rhs) {
      reset(rhs.mPtr);
      rhs.mPtr = nullptr;
    }
    return *this;
  }

  ~InferBuffer() { reset(); }

  explicit InferBuffer(size_t size) { mPtr = AllocatorType()(size, 0); }
  InferBuffer(size_t size, cudaStream_t stream) { 
      set_stream(stream);
      mPtr = AllocatorType()(size, mStream);
  }

  void set_stream(cudaStream_t stream) { mStream = stream; }
  void allocate(size_t size) {
    reset();
    mPtr = AllocatorType()(size, mStream);
  }

  void deallocate() {
    if (mPtr) {
      DeallocatorType()(mPtr, mStream);
    }
  }

  void reset(void* ptr = nullptr) {
    if (mPtr) {
      DeallocatorType()(mPtr, mStream);
    }
    mPtr = ptr;
  }

  void* get() const { return mPtr; }

  void set(void* ptr) { mPtr = ptr; }

 private:
  void* mPtr{nullptr};
  cudaStream_t mStream{0};
};

/// @brief This InferBufferPool template store the data for network's input and
/// output
///
/// @tparam AllocatorType
/// @tparam DeallocatorType
///
template <typename AllocatorType, typename DeallocatorType>
class InferBufferPool {
 public:
  InferBufferPool() = default;

  InferBufferPool(const InferBufferPool&) = delete;

  InferBufferPool& operator=(const InferBufferPool&) = delete;

  InferBufferPool(InferBufferPool&& rhs) {
    for (size_t i = 0; i < rhs.mPtrs.size(); ++i) {
      mPtrs[i] = rhs.mPtrs[i];
      rhs.mPtrs[i] = nullptr;
    }
  }

  InferBufferPool& operator=(InferBufferPool&& rhs) {
    if (this != &rhs) {
      for (size_t i = 0; i < rhs.mPtrs.size(); ++i) {
        mPtrs[i] = rhs.mPtrs[i];
        rhs.mPtrs[i] = nullptr;
      }
    }
    return *this;
  }

  ~InferBufferPool() { reset(); }

  explicit InferBufferPool(size_t size, size_t length = 1) {
    for (size_t i = 0; i < length; ++i) {
      mPtrs.emplace_back(AllocatorType()(size));
    }
  }

  void allocate(size_t size, size_t length = 1) {
    reset();
    for (size_t i = 0; i < length; ++i) {
      mPtrs.emplace_back(AllocatorType()(size));
    }
  }

  void deallocate() {
    for (size_t i = 0; i < mPtrs.size(); ++i) {
      if (mPtrs[i]) {
        DeallocatorType()(mPtrs[i]);
      }
    }
  }

  void reset(void* ptr = nullptr) {
    for (size_t i = 0; i < mPtrs.size(); i++) {
      void* mPtr = mPtrs[i];
      if (mPtr) {
        DeallocatorType()(mPtr);
      }
      mPtr = ptr;
    }
  }

  void* get(size_t index = 0) const { return mPtrs[index]; }

 private:
  std::vector<void*> mPtrs{nullptr};
};

}  // namespace inference
}  // namespace robosense

#endif
