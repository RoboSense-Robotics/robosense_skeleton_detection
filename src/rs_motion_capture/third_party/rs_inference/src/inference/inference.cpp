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
#include "hyper_vision/inference/inference.h"

#include "hyper_vision/common/log.h"

namespace robosense {
namespace inference {

InferEngine::Ptr createInferEngine(EngineType engine) {
  const auto& infer_engine = kInferEngine2NameMap.at(engine);
  if (!InferEngineRegister::isValid(infer_engine)) {
    INFER_ERROR << "Get illegal infer engine  " << infer_engine;
    exit(kFAILURE);
  }
  return std::shared_ptr<InferEngine>(
      InferEngineRegister::getInstanceByName(infer_engine));
}

InferEngine::Ptr createInferEngine(const std::string& engine) {
  if (kName2InferEngineMap.find(engine) != kName2InferEngineMap.end()) {
    const auto& engine_type = kName2InferEngineMap.at(engine);
    return createInferEngine(engine_type);
  } else {
    INFER_ERROR << "Get illegal engine type " << engine
                << ". Support engine type is";
    for (auto iter = kName2InferEngineMap.begin();
         iter != kName2InferEngineMap.end(); iter++) {
      INFER_ERROR << iter->first;
    }
    exit(kFAILURE);
  }
  return nullptr;
}

void InferEngine::setDebugLevel(DebugLevel level) noexcept {
  switch (level) {
    case DebugLevel::kTRACE: {
      RSLogger::setLoggingLevel(RSLogger::Level::Trace);
      break;
    }
    case DebugLevel::kDEBUG: {
      RSLogger::setLoggingLevel(RSLogger::Level::Debug);
      break;
    }
    case DebugLevel::kINFO: {
      RSLogger::setLoggingLevel(RSLogger::Level::Info);
      break;
    }
    case DebugLevel::kERROR: {
      RSLogger::setLoggingLevel(RSLogger::Level::Error);
      break;
    }
    default: {
      break;
    }
  }
}

}  // namespace inference
}  // namespace robosense
