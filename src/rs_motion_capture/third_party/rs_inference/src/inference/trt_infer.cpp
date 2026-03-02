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

#include "hyper_vision/inference/trt_infer.h"

#include "hyper_vision/inference/inference.h"
#include "hyper_vision/inference/trt_infer_impl.h"
#include "hyper_vision/inference/trt_utils.h"

namespace robosense {
namespace inference {

TrtInfer::TrtInfer() : impl(new TrtInferImpl) {}
TrtInfer::~TrtInfer() { delete impl; }

void TrtInfer::readModelFromFile(
    const ModelFileHandler& file_handler) noexcept {
  impl->readModelFromFile(file_handler);
}

void TrtInfer::init(const InferOptions& options) noexcept {
  impl->init(options);
}

void TrtInfer::reset() noexcept { impl->reset(); }

std::string TrtInfer::getModelVersion() const noexcept {
  return impl->getModelVersion();
}

std::map<std::string, std::string> TrtInfer::getModelInfos() const noexcept {
  return impl->getModelInfos();
}

std::string TrtInfer::getInputName(int32_t input_index) const noexcept {
  return impl->getInputName(input_index);
}

std::string TrtInfer::getOutputName(int32_t output_index) const noexcept {
  return impl->getOutputName(output_index);
}

void TrtInfer::forward() noexcept { impl->forward(); }
void TrtInfer::forward(const std::vector<void*>& inputs) noexcept {
  impl->forward(inputs);
}
void TrtInfer::forward(const std::vector<void*>& inputs,
                       const std::vector<void*>& outputs) noexcept {
  impl->forward(inputs, outputs);
}

void TrtInfer::transferInputToDevice() noexcept { impl->transferInputToDevice(); }
void TrtInfer::enqueue() noexcept { impl->enqueue(); }
void TrtInfer::transferOutputToHost() noexcept { impl->transferOutputToHost(); }
void TrtInfer::sync() noexcept { impl->cudaSteamSync(); }

int32_t TrtInfer::getNumInputs() const noexcept { return impl->getNumInputs(); }

int32_t TrtInfer::getNumOutputs() const noexcept {
  return impl->getNumOutputs();
}

cudaStream_t TrtInfer::getStream() const noexcept { return impl->getStream(); }
void TrtInfer::setStream(cudaStream_t stream) noexcept { impl->setStream(stream); }
int TrtInfer::getStreamPriority() const noexcept { return impl->getStreamPriority(); }

void* TrtInfer::getInputPtr(int32_t input_index, DeviceType device) noexcept {
  return impl->getInputPtr(input_index, device);
}

void* TrtInfer::getInputPtr(const std::string& name,
                            DeviceType device) noexcept {
  return impl->getInputPtr(name, device);
}

void* TrtInfer::getOutputPtr(int32_t output_index,
                             DeviceType device) const noexcept {
  return impl->getOutputPtr(output_index, device);
}

void* TrtInfer::getOutputPtr(const std::string& name,
                             DeviceType device) const noexcept {
  return impl->getOutputPtr(name, device);
}

size_t TrtInfer::getInputSize(int32_t input_index) const noexcept {
  return impl->getInputSize(input_index);
}

size_t TrtInfer::getInputSize(const std::string& name) const noexcept {
  return impl->getInputSize(name);
}

size_t TrtInfer::getOutputSize(int32_t output_index) const noexcept {
  return impl->getOutputSize(output_index);
}

size_t TrtInfer::getOutputSize(const std::string& name) const noexcept {
  return impl->getOutputSize(name);
}

Dims TrtInfer::getInputDims(int32_t input_index) const noexcept {
  return impl->getInputDims(input_index);
}

Dims TrtInfer::getOutputDims(int32_t output_index) const noexcept {
  return impl->getOutputDims(output_index);
}

float TrtInfer::getInferenceLatency() const noexcept {
  return impl->getInferenceLatency();
}

void TrtInfer::getProfilerInfo() const noexcept { impl->getProfilerInfo(); }

bool TrtInfer::setTensorAddress(const std::string& name,
                                void* address) noexcept {
  return impl->setTensorAddress(name, address);
}

void* TrtInfer::getTensorAddress(const std::string& name) noexcept {
  return impl->getTensorAddress(name);
}
void TrtInfer::changeTensorAddress(const std::string& src_name,
                                   const std::string& dst_name) noexcept {
  return impl->changeTensorAddress(src_name, dst_name);
}
void TrtInfer::changeTensorAddress(const std::string& src_name,
                                   const std::string& dst_name,
                                   void* src_device_ptr,
                                   void* src_host_ptr) noexcept {
  return impl->changeTensorAddress(src_name, dst_name, src_device_ptr,
                                   src_host_ptr);
}
}  // namespace inference
}  // namespace robosense
