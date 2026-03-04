//
// Created by sti on 2025/6/6.
//

#include "motion_capture/strategy/optical/two_stage/object_detection/object_detection.h"


namespace robosense {
namespace motion_capture {

void ObjectDetection::init(const YAML::Node& cfg_node) {
  spdlog::info("start init...");
  // init infer
  YAML::Node infer_node = cfg_node["infer"];
  std::string strategy = infer_node["strategy"].as<std::string>();
#ifdef __aarch64__
  std::string model_path = std::string(PROJECT_PATH) + "/" + infer_node[strategy]["engine_file_path"]["aarch"].as<std::string>();
#else
  std::string model_path = std::string(PROJECT_PATH) + "/" + infer_node[strategy]["engine_file_path"]["x86_64"].as<std::string>();
#endif
  spdlog::info("load model {}", model_path);
  inference::InferOptions init_options;
  init_options.model_format = inference::ModelFormat::kENGINE;
  init_options.save_path = model_path;
  init_options.use_cuda_graph = infer_node[strategy]["use_cuda_graph"].as<bool>();
  init_options.use_managed = infer_node[strategy]["use_managed_memory"].as<bool>();
  init_options.use_Unified_Address = infer_node[strategy]["use_unified_memory"].as<bool>();
  infer_ptr_ = inference::createInferEngine(inference::EngineType::kTENSORRT);
  infer_ptr_->init(init_options);
  infer_ptr_->setStream(stream_);
  // warm up
  infer_ptr_->forward();

  // init binding address
  bindings_ptr_->od_bindings_device.image = reinterpret_cast<float *>(infer_ptr_->getInputPtr("image", inference::DeviceType::kGPU));
  bindings_ptr_->od_bindings_host.output = reinterpret_cast<float *>(infer_ptr_->getOutputPtr("output", inference::DeviceType::kCPU));

  // init preprocess
  preprocess_ptr_ = std::make_shared<ObjectDetectionPreprocess>(bindings_ptr_, stream_);
  preprocess_ptr_->init(cfg_node["preprocess"]);

  // init postprocess
  postprocess_ptr_ = std::make_shared<ObjectDetectionPostprocess>(bindings_ptr_, stream_);
  postprocess_ptr_->init(cfg_node["postprocess"]);

  time_recorder_ptr_ = std::make_shared<TimeRecorder>(name());
  infer_time_recorder_ptr_ = std::make_shared<TimeRecorder>(name() + "Infer");
  spdlog::info("finish init.");
}

void ObjectDetection::process(const Msg::Ptr &msg_ptr) {
  time_recorder_ptr_->tic();
  spdlog::info("start process...");
  preprocess_ptr_->process(msg_ptr);
  infer_time_recorder_ptr_->tic();
  infer_ptr_->transferInputToDevice();
  infer_ptr_->forward();
  infer_ptr_->transferOutputToHost();
  cudaStreamSynchronize(stream_);
  infer_time_recorder_ptr_->toc();
  postprocess_ptr_->process(msg_ptr);
  spdlog::info("finish process.");
  time_recorder_ptr_->toc();
}


}
}

