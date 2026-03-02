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
#include <cuda_runtime_api.h>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <sstream>
#include "hyper_vision/inference/inference.h"
#include "hyper_vision/inference/utils.h"
#include "hyper_vision/common/log.h"

using namespace robosense::inference;

class InputParser{
public:
    InputParser(const int &argc, char **argv) {
        for (int i=1; i < argc; ++i) {
            this->tokens.push_back(std::string(argv[i]));
        }
    }
    const std::string& getCmdOption(const std::string &option) const {
        std::vector<std::string>::const_iterator itr;
        itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
        if (itr != this->tokens.end() && ++itr != this->tokens.end()) {
            return *itr;
        }
        static const std::string empty_string("");
        return empty_string;
    }
    bool cmdOptionExists(const std::string &option) const {
        return std::find(this->tokens.begin(), this->tokens.end(), option)
                != this->tokens.end();
    }

private:
    std::vector<std::string> tokens;
};

static void split_string(std::vector<std::string>& output, std::string input_str, char c) {
    std::stringstream str_stream(input_str);
    std::string segment;
    while (std::getline(str_stream, segment, c)) {
        output.push_back(segment);
    }
}

static void show_usage(std::string name) {
    std::cerr << "Usage: " << name << " <option(s)> SOURCES"
              << "Options:\n"
              << "\t--onnx\t\tinput onnx model, must specify\n"
              << "\t--engine\t\t"
              << "\t--encrypt_model\t\t0 for raw onnx model, 1 for encrypt onnx model, default is 0\n"
              << "\t--batch_size\t\tdefault is 1\n"
              << "\t--height\t\tdefault is the network input height\n"
              << "\t--width\t\tdefault is the network intput width\n"
              << "\t--mode\t\t0 for fp32 1 for fp16 2 for int8, default is 0\n"
              << "\t--calibrate_batch\t\tbatch size for calibration\n"
              << "\t--calibrate_data\t\tdata path for calibrate data which contain "
                 "point cloud feature files, default is empty\n"
              << "\t--calibrate_table\t\tcalibration table to store the scale\n"
              << "\t--max_workspace\t\tmax workspace to build engine, default is 30\n"
              << "\t--gpu\t\tchoose your device, default is 0\n"
              << "\t--dla\t\tset dla core if you want with 0,1..., default is -1(not enable)\n"
              << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        show_usage(argv[0]);
        return 1;
    }
    InputParser cmdparams(argc, argv);

    ModelFileHandler handler;
    handler.onnxFile = cmdparams.getCmdOption("--onnx");

    int encrypt_model = 0;
    const std::string& encrypt_model_str = cmdparams.getCmdOption("--encrypt_model");
    if (encrypt_model_str != "") {
        encrypt_model = std::stoi(encrypt_model_str);
    }
    if (encrypt_model) {
        handler.modelFormat = ModelFormat::kEncryptONNX;
    } else {
        handler.modelFormat = ModelFormat::kONNX;
    }

    int run_mode = 0;
    const std::string& run_mode_string = cmdparams.getCmdOption("--mode");
    if (run_mode_string != "") {
        run_mode = std::stoi(run_mode_string);
    }

    int batch_size = 1;
    const std::string& batch_size_string = cmdparams.getCmdOption("--batch_size");
    if (batch_size_string != "") {
        batch_size = std::stoi(batch_size_string);
    }

    int device_id = 0;
    const std::string& device_id_string = cmdparams.getCmdOption("--gpu");
    if (device_id_string != "") {
        device_id = std::stoi(device_id_string);
    }

    int cali_batch_size = 1;
    const std::string& cali_batch_size_str = cmdparams.getCmdOption("--calibrate_batch");
    if (cali_batch_size_str != "") {
        cali_batch_size = std::stoi(cali_batch_size_str);
    }

    int height = -1;
    const std::string& height_string = cmdparams.getCmdOption("--height");
    if (height_string != "") {
        height = std::stoi(height_string);
    }

    int width = -1;
    const std::string& width_string = cmdparams.getCmdOption("--width");
    if (width_string != "") {
        width = std::stoi(width_string);
    }

    int max_workspace = 30;
    const std::string& max_workspace_string = cmdparams.getCmdOption("--max_workspace");
    if (max_workspace_string != "") {
        max_workspace = std::stoi(max_workspace_string);
    }

    InferOptions options;
    options.batch_size = batch_size;
    options.device_id = device_id;

    std::cout << run_mode << std::endl;
    if (run_mode == 0) {
        options.enable_fp16 = false;
        options.enable_int8 = false;
    } else if (run_mode == 1) {
        options.enable_fp16 = true;
    } else if (run_mode == 2) {
        options.enable_int8 = true;
    }

    options.max_workspace = max_workspace;
    options.height = height;
    options.width = width;
    options.calibrate_batch = cali_batch_size;
    options.calibrate_data_path = cmdparams.getCmdOption("--calibrate_data");
    options.calibrate_cache_path = cmdparams.getCmdOption("--calibrate_table");
    options.keep_output_float = true;

    InferEngine::Ptr engine_ptr;

    engine_ptr = createInferEngine(EngineType::kTENSORRT);

    engine_ptr->readModelFromFile(handler);

    engine_ptr->init(options);

    return 0;
}
