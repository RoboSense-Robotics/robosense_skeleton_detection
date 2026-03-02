#ifndef HYPER_VISION_INFERENCE_TRT_PROFILER_H
#define HYPER_VISION_INFERENCE_TRT_PROFILER_H
#include <NvInferRuntime.h>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>
namespace robosense {
namespace inference {

//!
//! \struct LayerProfile
//! \brief Layer profile information
//!
struct LayerProfile {
  std::string name;
  std::vector<float> timeMs;
};

//!
//! \class Profiler
//! \brief Collect per-layer profile information, assuming times are reported in
//! the same order
//!
class Profiler : public nvinfer1::IProfiler {
 public:
  void reportLayerTime(char const* layerName, float timeMs) noexcept override;

  void print(std::ostream& os) const noexcept;

  //!
  //! \brief Export a profile to JSON file
  //!
  void exportJSONProfile(std::string const& fileName) const noexcept;

 private:
  float getTotalTime() const noexcept {
    auto const plusLayerTime = [](float accumulator, LayerProfile const& lp) {
      return accumulator + std::accumulate(lp.timeMs.begin(), lp.timeMs.end(),
                                           0.F, std::plus<float>());
    };
    return std::accumulate(mLayers.begin(), mLayers.end(), 0.0F, plusLayerTime);
  }

  float getMedianTime() const noexcept {
    if (mLayers.empty()) {
      return 0.F;
    }
    std::vector<float> totalTime;
    for (size_t run = 0; run < mLayers[0].timeMs.size(); ++run) {
      auto const layerTime = [&run](float accumulator, LayerProfile const& lp) {
        return accumulator + lp.timeMs[run];
      };
      auto t = std::accumulate(mLayers.begin(), mLayers.end(), 0.F, layerTime);
      totalTime.push_back(t);
    }
    return median(totalTime);
  }

  float getMedianTime(LayerProfile const& p) const noexcept {
    return median(p.timeMs);
  }

  static float median(std::vector<float> vals) {
    if (vals.empty()) {
      return 0.F;
    }
    std::sort(vals.begin(), vals.end());
    if (vals.size() % 2U == 1U) {
      return vals[vals.size() / 2U];
    }
    return (vals[vals.size() / 2U - 1U] + vals[vals.size() / 2U]) * 0.5F;
  }

  //! return the total runtime of given layer profile
  float getTotalTime(LayerProfile const& p) const noexcept {
    auto const& vals = p.timeMs;
    return std::accumulate(vals.begin(), vals.end(), 0.F, std::plus<float>());
  }

  float getAvgTime(LayerProfile const& p) const noexcept {
    return getTotalTime(p) / p.timeMs.size();
  }

  std::vector<LayerProfile> mLayers;
  std::vector<LayerProfile>::iterator mIterator{mLayers.begin()};
  int32_t mUpdatesCount{0};
};
}  // namespace inference
}  // namespace robosense

#endif
