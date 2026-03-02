#include "mono3d.h"
#include <cuda_runtime_api.h>

#include <iostream>
#include "hyper_vision/inference/trt_utils.h"
using robosense::inference::GPUAssert;

std::string formatDims(inference::Dims dims) {
    std::stringstream ss;
    int i;
    for (i=0; i<dims.nbDims-1; i++) {
        ss << dims.d[i] << "x";
        // if (dims.d[i+1]!=0) ss << "x";
    }
    ss << dims.d[i];
    return ss.str();
}
// void formatDims(inference::Dims dims) {
//     std::stringstream ss;
//     for (int i=0; i<dims.MAX_DIMS; i++) {
//         std::cout << dims.d[i] << "x";

//     }
//     // return ss;
// }

void printModelInfo(inference::InferEngine::Ptr infer_ptr) {
    for (auto i=0; i<infer_ptr->getNumInputs(); i++) {
        std::cout << "Input " << i
                  << " " << infer_ptr->getInputName(i)
                  << " " << formatDims(infer_ptr->getInputDims(i))
                  << std::endl;
                  
    }

    for (auto i=0; i< infer_ptr->getNumOutputs(); i++) {
        std::cout << "Output " << i 
                  << " " << infer_ptr->getOutputName(i)
                  << " " << formatDims(infer_ptr->getOutputDims(i))
                  << std::endl;

    }
}

void Mono3dDetection::init() {
    std::cout << name() << ": init" << std::endl;
    initInfer();
    // clean();
}

void Mono3dDetection::initInfer() {

    std::cout << name() << ": init infer" << std::endl;

    // step1. read model file
    infer_ptr_ = inference::createInferEngine(inference::EngineType::kTENSORRT);
    const auto &level = inference::DebugLevel::kDEBUG;
    infer_ptr_->setDebugLevel(level);

    // inference::ModelFileHandler handler;
    // handler.engineFile = "./test/mono3d_folded.trt";
    // handler.onnxFile = "./test/mono3d_folded.onnx";
    // handler.modelFormat = inference::ModelFormat::kENGINE;
    // handler.modelFormat = inference::ModelFormat::kONNX;
    // infer_ptr_->readModelFromFile(handler);

//     // step2. check model strategy, update unit_size and detect range, output other information
//     params_ptr_->checkModel(infer_ptr_->getModelInfos());

        // step3. init infer ptr
    inference::InferOptions init_options;
    // init_options.model_format = inference::ModelFormat::kONNX;
    init_options.save_path = "./test/mono3d_folded_fp32.trt";
    init_options.model_format = inference::ModelFormat::kENGINE;
    // init_options.max_workspace = 1;
    // init_options.batch_size = config_ptr->batch_size;
//     init_options.device_id = config_ptr->device_id;
    init_options.batch_size = 1;
    // init_options.height = 768;
    // init_options.width = 1152;
//     init_options.enable_int8 = config_ptr->enable_int_8;
//     init_options.enable_fp16 = config_ptr->enable_fp_16;
//     init_options.enable_dla = config_ptr->enable_dla;
    init_options.encrypt = false;
    infer_ptr_->init(init_options);    // must do this after checkModelInfos, because rows and cols is computed from model_info

    // step4. check model
    printModelInfo(infer_ptr_);
    //TODO return host ptr; 

    // step5. concat infer input and output

}

void Mono3dDetection::perception(std::vector<void*> inputs, std::vector<void*> outputs) {
    std::vector<void*> dev_input_ptrs{inputs.size(), nullptr}, dev_output_ptrs;

    // for (auto i=0; i < infer_ptr_->getNumInputs(); ++i) {
    //     dev_input_ptrs[i] = infer_ptr_->getInputPtr(i, inference::DeviceType::kGPU);
    //     BASE_CUDA_CHECK(cudaMemcpy(dev_input_ptrs[i], inputs[i], infer_ptr_->getInputSize(i), cudaMemcpyHostToDevice));
    // }
    // ((robosense::inference::Bindings*)infer_ptr_->getBindings())->transferInputToDevice();
    infer_ptr_->forward();
    // ((robosense::inference::Bindings*)infer_ptr_->getBindings())->transferOutputToHost();
    // for (auto i=0; i < infer_ptr_->getNumOutputs(); ++i) {
    //     infer_ptr_->getOutputBinds(i).buffer.trans();
    //     // BASE_CUDA_CHECK(cudaMemcpy(outputs[i], out_i, infer_ptr_->getOutputSize(i), cudaMemcpyDeviceToHost));
    // }
}
void Mono3dDetection::clean() {
    std::cout << name() << ": clean finished" << std::endl;
}

float iou(BBOX a, BBOX b) {
    float area1 = (a.x2 - a.x1 + 1) * (a.y2 - a.y1 + 1);
    float area2 = (b.x2 - b.x1 + 1) * (b.y2 - b.y1 + 1);
    int x11 = std::max(a.x1, b.x1);
    int y11 = std::max(a.y1, b.y1);
    int x22 = std::min(a.x2, b.x2);
    int y22 = std::min(a.y2, b.y2);
    float inter = (x22 - x11 + 1) * (y22 - y11 + 1);
    return inter / (area1 + area2 - inter);
}

std::vector<DetectionRes> nms(std::vector<DetectionRes>& input, float nms_thresh) {
    std::vector<DetectionRes> output;
    for (size_t i = 0; i < input.size(); ++i) {
            auto& item = input[i];
            output.push_back(item);
            for (size_t j = i + 1; j < input.size(); ++j) {
                if (iou(item.bbox, input[j].bbox) > nms_thresh) {
                    input.erase(input.begin()+j);
                    --j;
                }
            }
    }
    return output;
}
std::vector< std::vector<DetectionRes> >  multiclass_nms(std::vector< std::vector<DetectionRes> >& deteRes, int numClasses) {
    float score_thr = 0.5;
    float min_bbox_size = 5;
    float nms_pre = 1024;
    float iou_threshold = 0.4;
    float max_per_img = 96;
    
    std::vector< std::vector<DetectionRes> > outRes;
    for (int labeli=0; labeli<numClasses; ++labeli) {
        std::vector<DetectionRes> cur = deteRes[labeli];
        for (int i = cur.size()-1; i>=0; i--) {
            if (cur[i].score < score_thr  
                || (cur[i].bbox.x2 - cur[i].bbox.x1) < min_bbox_size 
                || (cur[i].bbox.y2 - cur[i].bbox.y1) < min_bbox_size) {
                cur.erase(cur.begin() + i);
            }
        }

        std::vector<DetectionRes> res;

        if (cur.size() > 0) {
            std::sort(cur.begin(), cur.end(), 
                [&](DetectionRes &x, DetectionRes& y) {return x.score > y.score;});

            std::vector<DetectionRes> topK;
            if (cur.size() > nms_pre) {
                topK = std::vector<DetectionRes>(cur.begin(), cur.begin() + nms_pre);
            } else {
                topK = std::vector<DetectionRes>(cur.begin(), cur.end());
            }

            res = nms(topK, iou_threshold);
        }
        outRes.emplace_back(res);
    }

    return outRes;
}