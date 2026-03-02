//
// Created by sti on 2025/6/9.
//

#include "motion_capture/interface/motion_capture_interface.h"

namespace robosense {
namespace motion_capture {

void MotionCaptureInterface::init(const YAML::Node& cfg_node) {
  std::string strategy;
  rally::yamlRead(cfg_node, "strategy", strategy);
  std::string glove_type = rally::ConfigureManager::getInstance().getCfgNode()["glove_type"].as<std::string>();
  if (strategy == "NoitonGloveInertialMotionCapture" && glove_type == "manus") {
    strategy = "ManusGloveInertialMotionCapture";
  }
  if (!BaseMotionCaptureRegister::isValid(strategy)) {
    const auto &all_instance = BaseMotionCaptureRegister::getAllInstances();
    AINFO << name() << ": support strategy as follows";
    for (size_t i = 0; i < all_instance.size(); ++i) {
      AINFO << all_instance[i]->name();
    }
  }
  motion_capture_impl_ptr_.reset(BaseMotionCaptureRegister::getInstanceByName(strategy));
  motion_capture_impl_ptr_->init(cfg_node[strategy]);
}

void MotionCaptureInterface::process(const Msg::Ptr &msg_ptr) {
  motion_capture_impl_ptr_->process(msg_ptr);
}

}
}
