#include "hyper_vision/inference/trt_profiler.h"
namespace robosense {
namespace inference {
void Profiler::reportLayerTime(char const* layerName, float timeMs) noexcept {
  if (mIterator == mLayers.end()) {
    bool const first = !mLayers.empty() && mLayers.begin()->name == layerName;
    mUpdatesCount += mLayers.empty() || first;
    if (first) {
      mIterator = mLayers.begin();
    } else {
      mLayers.emplace_back();
      mLayers.back().name = layerName;
      mIterator = mLayers.end() - 1;
    }
  }

  mIterator->timeMs.push_back(timeMs);
  ++mIterator;
}

void Profiler::print(std::ostream& os) const noexcept {
  std::string const nameHdr("Layer");
  std::string const timeHdr("   Time (ms)");
  std::string const avgHdr("   Avg. Time (ms)");
  std::string const medHdr("   Median Time (ms)");
  std::string const percentageHdr("   Time %");

  float const totalTimeMs = getTotalTime();

  auto const cmpLayer = [](LayerProfile const& a, LayerProfile const& b) {
    return a.name.size() < b.name.size();
  };
  auto const longestName =
      std::max_element(mLayers.begin(), mLayers.end(), cmpLayer);
  auto const nameLength =
      std::max(longestName->name.size() + 1, nameHdr.size());
  auto const timeLength = timeHdr.size();
  auto const avgLength = avgHdr.size();
  auto const medLength = medHdr.size();
  auto const percentageLength = percentageHdr.size();

  os << std::endl
     << "=== Profile (" << mUpdatesCount << " iterations ) ===" << std::endl
     << std::setw(nameLength) << nameHdr << timeHdr << avgHdr << medHdr
     << percentageHdr << std::endl;

  for (auto const& p : mLayers) {
    if (p.timeMs.empty() || getTotalTime(p) == 0.F) {
      // there is no point to print profiling for layer that didn't run at all
      continue;
    }
    // clang-format off
        os << std::setw(nameLength) << p.name << std::setw(timeLength) << std::fixed << std::setprecision(2) << getTotalTime(p)
           << std::setw(avgLength) << std::fixed << std::setprecision(4) << getAvgTime(p)
           << std::setw(medLength) << std::fixed << std::setprecision(4) << getMedianTime(p)
           << std::setw(percentageLength) << std::fixed << std::setprecision(1) << getTotalTime(p) / totalTimeMs * 100
           << std::endl;
    }
    {
        os << std::setw(nameLength) << "Total" << std::setw(timeLength) << std::fixed << std::setprecision(2)
           << totalTimeMs << std::setw(avgLength) << std::fixed << std::setprecision(4) << totalTimeMs / mUpdatesCount
           << std::setw(medLength) << std::fixed << std::setprecision(4) << getMedianTime()
           << std::setw(percentageLength) << std::fixed << std::setprecision(1) << 100.0 << std::endl;
    // clang-format on
  }
  os << std::endl;
}

void Profiler::exportJSONProfile(std::string const& fileName) const noexcept {
  std::ofstream os(fileName, std::ofstream::trunc);
  os << "[" << std::endl
     << "  { \"count\" : " << mUpdatesCount << " }" << std::endl;

  auto const totalTimeMs = getTotalTime();

  for (auto const& l : mLayers) {
    // clang-format off
        os << ", {" << " \"name\" : \""      << l.name << "\""
                       ", \"timeMs\" : "     << getTotalTime(l)
           <<          ", \"averageMs\" : "  << getAvgTime(l)
           <<          ", \"medianMs\" : "  << getMedianTime(l)
           <<          ", \"percentage\" : " << getTotalTime(l) / totalTimeMs * 100
           << " }"  << std::endl;
    // clang-format on
  }
  os << "]" << std::endl;
}

}  // namespace inference
}  // namespace robosense