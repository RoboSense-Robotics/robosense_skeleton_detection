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

#include "hyper_vision/inference/trt_bindings.h"

namespace robosense {
namespace inference {

void Bindings::addBinding(TensorInfo const& tensorInfo) {
  auto const b = tensorInfo.bindingIndex;
  while (mBindings.size() <= static_cast<size_t>(b)) {
    mBindings.emplace_back();
    mDevicePointers.emplace_back();
    // mHostPointers.emplace_back();
  }
  mNames[tensorInfo.name] = b;
  mBindings[b].name = tensorInfo.name;
  mBindings[b].isInput = tensorInfo.isInput;
  mBindings[b].volume = tensorInfo.vol;
  mBindings[b].dataType = tensorInfo.dataType;
  if (tensorInfo.isDynamic) {
    // Only output shape can be possibly unknown because of DDS.
    if (!tensorInfo.isInput) {
      INFER_ERROR << "Only output shape can be possibly unknown";
      exit(kFAILURE);
    }
    if (mBindings[b].outputAllocator == nullptr) {
      if (mUseUnifiedAddress) {
        mBindings[b].outputAllocator.reset(
            new OutputAllocator(new UnifiedAddressBuffer(mStream)));
      } else if (mUseManaged) {
        mBindings[b].outputAllocator.reset(
            new OutputAllocator(new UnifiedMirroredBuffer(mStream)));
      } else {
        mBindings[b].outputAllocator.reset(
            new OutputAllocator(new DiscreteMirroredBuffer(mStream)));
      }
    }
  } else {
    if (mBindings[b].buffer == nullptr) {
      if (tensorInfo.isInput) {
        mNumInput_ ++;
        if (mUseUnifiedAddress) {
          mBindings[b].buffer.reset(new UnifiedAddressBuffer(mStream));
        } else if (mUseManaged) {
          mBindings[b].buffer.reset(new UnifiedMirroredBuffer(mStream));
        } else if (mUseGPUInput) {
          mBindings[b].buffer.reset(new GPUDeviceBuffer(mStream));
        } else {
          mBindings[b].buffer.reset(new DiscreteMirroredBuffer(mStream));
        }
      } else {  // output
        if (mUseUnifiedAddress) {
          mBindings[b].buffer.reset(new UnifiedAddressBuffer(mStream));
        } else if (mUseManaged) {
          mBindings[b].buffer.reset(new UnifiedMirroredBuffer(mStream));
        } else if (mUseGPUInput) {
          mBindings[b].buffer.reset(new DiscreteMirroredBuffer(mStream));
        } else {
          mBindings[b].buffer.reset(new DiscreteMirroredBuffer(mStream));
        }
      }
      // Some memory allocators return nullptr when allocating zero bytes, but
      // TensorRT requires a non-null ptr even for empty tensors, so allocate a
      // dummy byte.
      if (tensorInfo.vol == 0) {
        mBindings[b].buffer->allocate(1);
      } else {
        mBindings[b].buffer->allocate(
            static_cast<size_t>(tensorInfo.vol) *
            static_cast<size_t>(dataTypeSize(tensorInfo.dataType)));
      }
      mDevicePointers[b] = mBindings[b].buffer->getDeviceBuffer();
      // mHostPointers[b] = mBindings[b].buffer->getHostBuffer();
    }
  }
}

// void** Bindings::getDeviceBuffers() { return mDevicePointers.data(); }

// void** Bindings::getHostBuffers() { return mHostPointers.data(); }

void Bindings::transferInputToDevice() {
  for (auto& b : mNames) {
    if (mBindings[b.second].isInput) {
      mBindings[b.second].buffer->hostToDevice();
    }
  }
}

void Bindings::transferOutputToHost() {
  for (auto& b : mNames) {
    // if (b.first != "velocity") continue;
    if (!mBindings[b.second].isInput) {
      if (mBindings[b.second].outputAllocator != nullptr) {
        mBindings[b.second].outputAllocator->getBuffer()->deviceToHost();
      } else {
        mBindings[b.second].buffer->deviceToHost();
      }
    }
  }
}

const Binding& Bindings::getBinding(const std::string& name) const {
  if (mNames.find(name) != mNames.end()) {
    return mBindings[mNames.at(name)];
  } else {
    INFER_ERROR << "Get illegal binding name " << name;
    exit(kFAILURE);
  }
}

const Binding& Bindings::getInputBinding(int32_t input_index) const {
  if (input_index >= static_cast<int32_t>(mNumInput_)) {
    INFER_ERROR << "Input index should be less than the number of inputs";
    exit(kFAILURE);
  } else {
    return mBindings[input_index];
  }
}

const Binding& Bindings::getOutputBinding(int32_t output_index) const {
  auto index = output_index + mNumInput_;
  if (index >= static_cast<int32_t>(mBindings.size())) {
    INFER_ERROR << "Output index should be less than the number of outputs";
    exit(kFAILURE);
  }
  return mBindings[index];
}

std::unordered_map<std::string, int> Bindings::getBindings(
    std::function<bool(Binding const&)> predicate) const {
  std::unordered_map<std::string, int> bindings;
  for (auto const& n : mNames) {
    auto const binding = n.second;
    if (predicate(mBindings[binding])) {
      bindings.insert(n);
    }
  }
  return bindings;
}

bool Bindings::setTensorAddresses(nvinfer1::IExecutionContext& context) const {
  for (auto const& b : mNames) {
    auto const name = b.first.c_str();
    auto const location = context.getEngine().getTensorLocation(name);
    if (location == nvinfer1::TensorLocation::kDEVICE) {
      if (mBindings[b.second].outputAllocator != nullptr) {
        if (!context.setOutputAllocator(
                name, mBindings[b.second].outputAllocator.get())) {
          return false;
        }
      } else {
        if (!context.setTensorAddress(name, mBindings[b.second].buffer->getDeviceBuffer())) {
          return false;
        }
      }
    }
  }
  return true;
}

}  // namespace inference
}  // namespace robosense
