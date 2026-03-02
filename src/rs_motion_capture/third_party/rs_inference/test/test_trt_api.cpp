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

#include <unistd.h>
#include <cuda_runtime_api.h>
#include <iostream>
#include <chrono>
#include "hyper_vision/inference/inference.h"
#include "hyper_vision/inference/utils.h"
#include "hyper_vision/common/log.h"

static void show_usage(std::string name) {
    std::cerr << "Usage: " << name << " model" << std::endl;
}

// using namespace robosense::inference;

int main(int argc, char** argv) {
    // if (argc < 2) {
    //     show_usage(argv[0]);
    //     return 1;
    // }
    // int count = 0;
    // if (cudaSuccess != cudaGetDeviceCount(&count)) {
    //     return -1;
    // }
    // if (count == 0) {
    //     return -1;
    // }
    // for (int device = 0; device < count; ++device) {
    //     cudaDeviceProp prop;
    //     if (cudaSuccess == cudaGetDeviceProperties(&prop, device)) {
    //         std::printf("Device Name: %s\n", prop.name);
    //         std::printf("Memory: %ld\n",prop.totalConstMem);
    //         std::printf("Compute Capacity: %d.%d\n", prop.major, prop.minor);
    //     }
    // }

    // std::string model_file(argv[1]);
    // std::string feature_file(argv[2]);

    // ModelFileHandler handler;
    // InferOptions options;
    // options.batch_size = 1;
    // options.device_id = 0;

    // // options.height = 224;
    // // options.width = 224;
    // // options.enable_dla = false;
    // // options.enable_fp16 = false;
    // // options.enable_int8 = true;
    // // options.max_workspace = 30;
    // options.height = 416;
    // options.width = 512;
    // // options.calibrate_batch = 2;
    // // options.calibrate_cache_path = "CalibrationTable";
    // // options.keep_output_float = true;

    // handler.onnxFile = model_file;
    // handler.engineFile = "";
    // handler.modelFormat = ModelFormat::kEncryptONNX;

    // InferEngine::Ptr engine_ptr;
    // engine_ptr = createInferEngine("TensorRT");

    // engine_ptr->setDebugLevel(DebugLevel::kDEBUG);

    // engine_ptr->readModelFromFile(handler);

    //     // test get model info
    // auto model_info = engine_ptr->getModelInfos();
    // for (auto iter = model_info.begin(); iter != model_info.end(); iter++) {
    //     INFER_DEBUG << iter->first << " : "<< iter->second;
    // }
    // engine_ptr->init(options);

    // // test get number input
    // INFER_DEBUG << "Number of input:  " << engine_ptr->getNumInputs();

    // // test get number output
    // INFER_DEBUG << "Number of output:  " << engine_ptr->getNumOutputs();

    // // test get input name
    // INFER_DEBUG << "Input name:  " << engine_ptr->getInputName(0);

    // // test get output name
    // INFER_DEBUG << "Output name:  " << engine_ptr->getOutputName(0);

    // // test get input size
    // INFER_DEBUG << "Input size:  " << engine_ptr->getInputSize(0);

    // // test get output size
    // INFER_DEBUG << "Output size:  " << engine_ptr->getOutputSize(0);

    // // test get model version
    // Dims in_dim = engine_ptr->getInputDims(0);

    // DeviceType cpu = DeviceType::kCPU;
    // DeviceType gpu = DeviceType::kGPU;

    // void* input_ptr = engine_ptr->getInputPtr(0, cpu);
    // void* output_ptr = engine_ptr->getOutputPtr(0, cpu);
    // // FILE *f = fopen(feature_file.c_str(), "rb");
    // // fread(input_ptr, 1, engine_ptr->getInputSize(0), f);
    // // fclose(f);
    // if (input_ptr == nullptr) {
    //     INFER_WARN << "input_ptr == nullptr";
    // }

    // for (int i = 0; i < 200; i++) {
    //     // std::cout << "Inference - " << i << std::endl;
    //     INFER_DEBUG << "Inference - "<< i;
    //     auto start_time = std::chrono::high_resolution_clock::now();

    //     engine_ptr->forward();
    //     auto end_time = std::chrono::high_resolution_clock::now();
    //     float latency_ = std::chrono::duration<float, std::milli>(end_time - start_time).count();
    //     INFER_DEBUG << "latency:  " << latency_ << "ms";
    // }

    // // std::cout << "avg: " << latency / 1000 << "ms" << std::endl;
    // // FILE *fout = fopen("tensorrt_output.bin", "wb");
    // // fwrite(output_ptr, 1, engine_ptr->getOutputSize(0), fout);
    // // fclose(fout);

    // INFER_DEBUG << "Test Done!";
    return 0;
}
