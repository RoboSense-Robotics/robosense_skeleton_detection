//
// Created by sti on 2025/6/10.
//

#ifndef MOTION_CAPTURE_DATA_STRUCTURE_BBOX_H
#define MOTION_CAPTURE_DATA_STRUCTURE_BBOX_H

#include <algorithm>
#include <iostream>

namespace robosense {
namespace motion_capture {

template <typename T>
class BBox {
public:
  BBox() {}

  BBox(T xmax, T xmin, T ymax, T ymin, float score)
      : xmax(xmax), xmin(xmin), ymax(ymax), ymin(ymin), score(score) {
  }

  BBox(const BBox& other)
      : xmax(other.xmax), xmin(other.xmin), ymax(other.ymax), ymin(other.ymin), score(other.score) {
  }

  float iou(const BBox& other) const {
    T inter_xmin = std::max(xmin, other.xmin);
    T inter_ymin = std::max(ymin, other.ymin);
    T inter_xmax = std::min(xmax, other.xmax);
    T inter_ymax = std::min(ymax, other.ymax);
    T inter_area = std::max(T(0), inter_xmax - inter_xmin) * std::max(T(0), inter_ymax - inter_ymin);
    T area1 = (xmax - xmin) * (ymax - ymin);
    T area2 = (other.xmax - other.xmin) * (other.ymax - other.ymin);
    return static_cast<float>(inter_area) / (area1 + area2 - inter_area + 1e-6f);
  }

  // 重载<<
  friend std::ostream& operator<<(std::ostream& os, const BBox& bbox) {
    os << "BBox(" << bbox.xmin << ", " << bbox.ymin << ", " << bbox.xmax << ", " << bbox.ymax
       << ", score: " << bbox.score << ")";
    return os;
  }

  T xmax, xmin, ymax, ymin;
  float score;

};

}
}

#endif //MOTION_CAPTURE_DATA_STRUCTURE_BBOX_H
