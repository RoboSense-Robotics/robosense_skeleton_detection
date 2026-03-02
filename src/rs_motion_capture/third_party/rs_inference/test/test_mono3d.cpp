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
#include "mono3d.h"
#include <opencv2/opencv.hpp>

static void show_usage(std::string name) {
    std::cerr << "Usage: " << name << " model" << std::endl;
}

// using namespace robosense::inference;
cv::Mat PreprocessImage(const std::string& imgPath, int h, int w) {
    cv::Mat img = cv::imread(imgPath);
    std::cout << img.size << std::endl;
    // resize
    // cv::resize(img, img, cv::Size(1152, 768));
    // cv::imshow("test", img);
    // cv::waitKey(0);
    // img.convertTo(img, CV_32FC3);
    cv::Scalar mean(102.9801, 115.9465, 122.7717);
    cv::Mat blob = cv::dnn::blobFromImage(img, 1.0, cv::Size(w, h), mean, false, false);

    std::cout << blob.size << std::endl;
    return blob;
}
int main(int argc, char** argv) {
    // std::string imgPath = std::string(argv[2]);
    std::string imgPath = std::string("test/1682128778300004096.jpeg");
    cv::Mat inputBlob = PreprocessImage(imgPath, 768, 1152);

    Mono3dDetection mono3d;
    mono3d.init();


    std::vector<void *> inputs;
    std::vector<void *> outputs {10, nullptr};

    inputs.emplace_back(mono3d.getInferEngine()->getInputPtr(0, robosense::inference::DeviceType::kCPU));
    std::memcpy(inputs[0], inputBlob.data, mono3d.getInferEngine()->getInputSize(0) * sizeof(float));

    mono3d.perception(inputs, outputs);

    for (size_t i=0; i<outputs.size(); ++i) {
        outputs[i] = mono3d.getInferEngine()->getOutputPtr(i, robosense::inference::DeviceType::kCPU);
    }
    std::vector< std::vector<DetectionRes> > deteRes, outRes;
    for (int i=0; i<8; ++i) {
        std::vector<DetectionRes> classRes;
        for (int j=0; j<18360; ++j) {
            DetectionRes res;

            int bias = i * 18360 + j;
            float* ptr = (float*)(outputs[0]) + (bias*4);
            // std::memcpy(res.bbox, ptr, sizeof(float)*4);
            res.bbox = BBOX( ptr[0], ptr[1], ptr[2], ptr[3] );

            // ptr = (float*)(mono3d_params_.vBufferH[2]) + (bias*4);
            // res.full_bbox = BBOX( ptr[0], ptr[1], ptr[2], ptr[3] );

            ptr = (float*)(outputs[2]) + bias;
            res.score = *(ptr);
            // ptr = (float*)(mono3d_params_.vBufferH[4]) + bias;
            // res.label = *(ptr);

            // ptr = (float*)(mono3d_params_.vBufferH[5]) + (bias*2);
            // res.alpha = ALPHA( ptr[0], ptr[1] );

            // ptr = (float*)(mono3d_params_.vBufferH[6]) + bias;
            // res.depth = *(ptr);

            // ptr = (float*)(mono3d_params_.vBufferH[7]) + (bias*4);
            // res.size_long = SIZE( ptr[0], ptr[1], ptr[2], ptr[3] );
            // ptr = (float*)(mono3d_params_.vBufferH[8]) + (bias*4);
            // res.size_short = SIZE( ptr[0], ptr[1], ptr[2], ptr[3] );

            // ptr = (float*)(mono3d_params_.vBufferH[9]) + (bias*3);
            // res.edge_validity = EDGE_VALIDITY( ptr[0], ptr[1], ptr[2] );

            // bias = j;
            // ptr = (float*)(mono3d_params_.vBufferH[10]) + (bias*2);
            // res.points = POINT( ptr[0], ptr[1] );

            classRes.emplace_back(res);
        }
        deteRes.emplace_back(classRes);
    }
    auto time_05 = std::chrono::steady_clock::now();
    // std::cout << "construct res: " 
    //     << std::chrono::duration_cast<std::chrono::milliseconds>(time_05 - time_04).count() 
    //     << " ms" << std::endl;
    // for (int i=0;i<3;i++) {
    //     std::cout << deteRes[0][i].bbox.x1 << " "
    //               << deteRes[0][i].bbox.y1 << " "
    //               << deteRes[0][i].bbox.x2 << " "
    //               << deteRes[0][i].bbox.y2 << " "
    //               << deteRes[0][i].score << " "
    //               << std::endl;
    // }
    // for (int i=18144-3;i<18144;i++) {
    //     std::cout << deteRes[0][i].bbox.x1 << " "
    //               << deteRes[0][i].bbox.y1 << " "
    //               << deteRes[0][i].bbox.x2 << " "
    //               << deteRes[0][i].bbox.y2 << " "
    //               << deteRes[0][i].score << " "
    //               << std::endl;
    // }
    // outRes = multiclass_nms_gpu(deteRes, 8);
    outRes = multiclass_nms(deteRes, 8);
    auto time_06 = std::chrono::steady_clock::now();
    std::cout << "postprocess(nms) res: " 
        << std::chrono::duration_cast<std::chrono::milliseconds>(time_06 - time_05).count() 
        << " ms" << std::endl;

    for (size_t i=0; i<outRes.size(); ++i) {
        auto classesRes =  outRes[i];
        std::cout << i <<std::endl;
        for (size_t j=0; j< classesRes.size(); ++j) {
            std::cout << classesRes[j].bbox.x1 << " " 
                      << classesRes[j].bbox.y1 << " "
                      << classesRes[j].bbox.x2 << " "
                      << classesRes[j].bbox.y2 << std::endl;
        }
        std::cout << "===============" << std::endl;
    }
    return 0;
}
