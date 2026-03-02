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

#ifndef HYPER_VISION_INFERENCE_TRT_INFER_H
#define HYPER_VISION_INFERENCE_TRT_INFER_H

#include "hyper_vision/inference/inference.h"

namespace robosense {
namespace inference {

class TrtInferImpl;
class TrtInfer final : public InferEngine {
 public:
  using Ptr = std::shared_ptr<TrtInfer>;

  void readModelFromFile(
      const ModelFileHandler& file_handler) noexcept override;

  void init(const InferOptions& options) noexcept override;

  void reset() noexcept override;

  std::string getModelVersion() const noexcept override;

  std::map<std::string, std::string> getModelInfos() const noexcept override;

  std::string getInputName(int32_t input_index) const noexcept override;

  std::string getOutputName(int32_t output_index) const noexcept override;

  void forward() noexcept override;
  void forward(const std::vector<void*>& input) noexcept override;
  void forward(const std::vector<void*>& input,
               const std::vector<void*>& output) noexcept override;
  void transferInputToDevice() noexcept override;
  void enqueue() noexcept override;
  void transferOutputToHost() noexcept override;
  void sync() noexcept;

  int32_t getNumInputs() const noexcept override;

  int32_t getNumOutputs() const noexcept override;

  cudaStream_t getStream() const noexcept override;
  int getStreamPriority() const noexcept override;
  void setStream(cudaStream_t stream) noexcept override;

  void* getInputPtr(int32_t input_index, DeviceType device) noexcept override;

  void* getInputPtr(const std::string& name,
                    DeviceType device) noexcept override;

  void* getOutputPtr(int32_t output_index,
                     DeviceType device) const noexcept override;

  void* getOutputPtr(const std::string& name,
                     DeviceType device) const noexcept override;

  size_t getInputSize(int32_t input_index) const noexcept override;
  size_t getInputSize(const std::string& name) const noexcept override;

  size_t getOutputSize(int32_t output_index) const noexcept override;
  size_t getOutputSize(const std::string& name) const noexcept override;

  Dims getInputDims(int32_t intput_index) const noexcept override;

  Dims getOutputDims(int32_t output_index) const noexcept override;

  float getOutputScale(int32_t output_index) const noexcept override {
    return 1.0;
  }
  bool setTensorAddress(const std::string& name,
                        void* address) noexcept override;
  void* getTensorAddress(const std::string& name) noexcept override;
  void changeTensorAddress(const std::string& src_name,
                           const std::string& dst_name) noexcept override;
  void changeTensorAddress(const std::string& src_name,
                           const std::string& dst_name, void* src_device_ptr,
                           void* src_host_ptr) noexcept override;
  float getInferenceLatency() const noexcept override;
  void getProfilerInfo() const noexcept;
  TrtInfer();
  TrtInfer(const TrtInfer&) = delete;
  TrtInfer& operator=(const TrtInfer&) = delete;
  ~TrtInfer();

 private:
  TrtInferImpl* impl;
};

RS_REGISTER_INFER_ENGINE(TrtInfer);

}  // namespace inference
}  // namespace robosense

#endif
