//
// Created by sti on 2025/6/10.
//

#ifndef MOTION_CAPTURE_DATA_STRUCTURE_POINT_H
#define MOTION_CAPTURE_DATA_STRUCTURE_POINT_H

#include <cmath>

namespace robosense {
namespace motion_capture {

template <typename T>
class Point2 {
public:
  Point2() : x(0), y(0), score(0.f), valid(false) {};
  Point2(T x, T y) : x(x), y(y), score(0.f), valid(false) {}
  Point2(T x, T y, float score, bool valid) : x(x), y(y), score(score), valid(valid) {}

  bool is_valid() const {
    return valid;
  }

  float distance(const Point2& other) const {
    return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
  }

  T x, y;
  float score;
  bool valid;
};

template <typename T>
class Point3 {
public:
  Point3() : x(0), y(0), z(0), score(0.f), valid(false) {};
  Point3(T x, T y, T z) : x(x), y(y), z(z), score(0.f) {}
  Point3(T x, T y, T z, float score, bool valid) : x(x), y(y), z(z), score(score), valid(valid) {}

  bool is_valid() const {
    return valid;
  }

  float distance(const Point3& other) const {
    return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y) + (z - other.z) * (z - other.z));
  }

  T x, y, z;
  float score;
  bool valid;
};

typedef Point2<int> Point2i;
typedef Point2<float> Point2f;
typedef Point2<double> Point2d;
typedef Point3<int> Point3i;
typedef Point3<float> Point3f;
typedef Point3<double> Point3d;


}
}

#endif //MOTION_CAPTURE_DATA_STRUCTURE_POINT_H
