//
// Created by sti on 2025/6/10.
//

#ifndef MOTION_CAPTURE_ALGORITHM_GEOMETRY_NMS_H
#define MOTION_CAPTURE_ALGORITHM_GEOMETRY_NMS_H

#include <cstdint>
#include <vector>
#include <algorithm>
#include "motion_capture/data_structure/bbox.h"

namespace robosense {
namespace motion_capture {

template <typename T>
std::vector<BBox<T>> nms(std::vector<BBox<T>> boxes, float thresh) {
  std::vector<BBox<T>> res;
  if (boxes.empty()) {
    return res;
  }
  std::sort(boxes.begin(), boxes.end(), [](const BBox<T>& a, const BBox<T>& b) {
    return a.score > b.score;
  });
  std::vector<uint8_t> suppressed(boxes.size(), 0);
  for (size_t i = 0; i < boxes.size(); ++i) {
    if (suppressed[i] == 1) {
      continue;
    }
    res.push_back(boxes[i]);
    for (size_t j = i + 1; j < boxes.size(); ++j) {
      if (suppressed[j] == 1) {
        continue;
      }
      float iou = boxes[i].iou(boxes[j]);
      if (iou > thresh) {
        suppressed[j] = 1;
      }
    }
  }
  return res;
}

}
}

#endif //MOTION_CAPTURE_ALGORITHM_GEOMETRY_NMS_H
