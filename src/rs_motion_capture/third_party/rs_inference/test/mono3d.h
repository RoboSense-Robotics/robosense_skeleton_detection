
#pragma once
#include <memory>
#include <algorithm>
// #include "hyper_vision/inference/trt_utils.h"
#include "hyper_vision/inference/inference.h"

using namespace robosense;
struct BBOX {
    BBOX(){}
    BBOX(float x1, float y1, float x2, float y2)
    :x1(x1), y1(y1), x2(x2), y2(y2) {}
    float x1, x2, y1, y2;
};
struct ALPHA {
    ALPHA(){}
    ALPHA(float x, float y): x(x), y(y) {}
    float x, y;
};
struct SIZE {
    SIZE(){}
    SIZE(float a, float b, float c, float d):a(a), b(b), c(c),d(d) {}
    float a,b,c,d;
};
struct EDGE_VALIDITY {
    EDGE_VALIDITY(){}
    EDGE_VALIDITY(float x, float y, float z): x(x), y(y), z(z) {}
    float x,y,z;
};
struct POINT {
    POINT(){}
    POINT(float x, float y): x(x), y(y) {}
    float x, y;
};
struct DetectionRes {
    DetectionRes(): bbox(0,0,0,0) {}
    BBOX bbox;
    BBOX full_bbox;
    float score;
    float label;
    ALPHA alpha;
    float depth;
    SIZE size_long;
    SIZE size_short;
    EDGE_VALIDITY edge_validity;
    POINT points;
};
class Mono3dDetection {
public:
    using Ptr = std::unique_ptr<Mono3dDetection>;

    Mono3dDetection() {}

    void init();

    void perception(std::vector<void*> inputs, std::vector<void*> outputs);

    inference::InferEngine::Ptr getInferEngine() {
        return infer_ptr_;
    }
private:
    const std::string name() const {
        return "Mono3dDetection";
    }

    void initInfer();
    // void infer();

    // void initPreprocess();

    // void initPostprocess();

    // clean member variables
    void clean();

    // GalaxyParams::Ptr params_ptr_;
    // GalaxyPreprocess::Ptr preprocess_ptr_;
    // Postprocess::Ptr postprocess_ptr_;
    inference::InferEngine::Ptr infer_ptr_;

// #ifdef COMPILE_WITH_CUDA

//     GalaxyCudaPreprocess::Ptr preprocess_cuda_ptr_;
//     GalaxyCudaPostprocess::Ptr postprocess_cuda_ptr_;

// #endif  // COMPILE_WITH_CUDA

    // std::future<void> void_results_;
    bool init_ = false;
};


std::vector< std::vector<DetectionRes> >  multiclass_nms(std::vector< std::vector<DetectionRes> >& deteRes, int numClasses);