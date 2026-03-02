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
#ifndef RALLY_CORE_ALGORITHM_GEOMETRY_IMPL_POLYGON_H
#define RALLY_CORE_ALGORITHM_GEOMETRY_IMPL_POLYGON_H

namespace rally {

template<typename PointT>
Polygon2d::Polygon2d(std::vector<PointT> points) {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    points_.resize(points.size());
    for (size_t i{0}; i < points.size(); ++i) {
        points_[i].x = points[i].x;
        points_[i].y = points[i].y;
    }
    buildFromPoints();
}

template<typename PointT>
float Polygon2d::distanceToBoundary(const PointT &point) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    float distance = std::numeric_limits<float>::infinity();
    for (uint32_t i{0}; i < num_points_; ++i) {
        distance = std::min(distance, line_segments_[i].distanceTo(point));
    }
    return distance;
}

template<typename PointT>
float Polygon2d::distanceTo(const PointT &point) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    RENSURE(points_.size() >= 3U);
    if (isPointIn(point)) {
        return 0.f;
    }
    float distance = std::numeric_limits<float>::infinity();
    for (uint32_t i{0}; i < num_points_; ++i) {
        distance = std::min(distance, line_segments_[i].distanceTo(point));
    }
    return distance;
}

template<typename PointT>
float Polygon2d::distanceSquareTo(const PointT &point) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    RENSURE(points_.size() >= 3U);
    if (isPointIn(point)) {
        return 0.f;
    }
    float distance_sqr = std::numeric_limits<float>::infinity();
    for (uint32_t i{0}; i < num_points_; ++i) {
        distance_sqr = std::min(distance_sqr, line_segments_[i].distanceSquareTo(point));
    }
    return distance_sqr;
}

template<typename PointT>
bool Polygon2d::isPointIn(const PointT &point) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    RENSURE(points_.size() >= 3U);
    if (isPointOnBoundary(point)) {
        return true;
    }
    uint32_t j = num_points_ - 1;
    uint32_t c = 0;
    for (uint32_t i{0}; i < num_points_; ++i) {
        if ((points_[i].y > point.y) != (points_[j].y > point.y)) {
            const float side = crossProd(point, points_[i], points_[j]);
            if (points_[i].y < points_[j].y ? side > 0.f : side < 0.f) {
                ++c;
            }
        }
        j = i;
    }
    return c & 1;
}

template<typename PointT>
bool Polygon2d::isPointOnBoundary(const PointT &point) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    RENSURE(points_.size() >= 3U);
    return std::any_of(line_segments_.begin(), line_segments_.end(),
                       [&](const LineSegment2d &poly_seg) { return poly_seg.isPointIn(point); });
}

template<typename PointT>
bool Polygon2d::computeConvexHull(const std::vector<PointT> &points, Polygon2d &polygon) {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    const uint32_t n = static_cast<uint32_t>(points.size());
    if (n < 3) {
        return false;
    }
    std::vector<uint32_t> sorted_indices(n);
    for (uint32_t i = 0; i < n; ++i) {
        sorted_indices[i] = i;
    }
    std::sort(sorted_indices.begin(), sorted_indices.end(),
              [&](const int idx1, const int idx2) {
                  const auto &pt1 = points[idx1];
                  const auto &pt2 = points[idx2];
                  const float dx = pt1.x - pt2.x;
                  if (std::abs(dx) > R_F_EPS) {
                      return dx < 0.f;
                  }
                  return pt1.y < pt2.y;
              });
    uint32_t count = 0;
    std::vector<uint32_t> results;
    results.reserve(n);
    uint32_t last_count = 1;
    for (uint32_t i = 0; i < n + n; ++i) {
        if (i == n) {
            last_count = count;
        }
        const auto idx = sorted_indices[(i < n) ? i : (n + n - 1 - i)];
        const auto &pt = points[idx];
        while (count > last_count &&
               crossProd(points[results[count - 2]], points[results[count - 1]], pt) <= R_F_EPS) {
            results.pop_back();
            --count;
        }
        results.push_back(idx);
        ++count;
    }
    --count;
    if (count < 3) {
        return false;
    }
    std::vector<PointT> result_points;
    result_points.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        result_points.push_back(points[results[i]]);
    }
    polygon = Polygon2d(result_points);
    return true;
}

template<typename PointT>
void Polygon2d::getAllVertices(std::vector<PointT> &vertices) const {
    vertices.resize(points_.size());
    for (size_t i = 0; i < points_.size(); ++i) {
        vertices[i].x = points_[i].x;
        vertices[i].y = points_[i].y;
    }
}

inline std::ostream &operator<<(std::ostream &os, const Polygon2d &p) {
    os << p.infos();
    return os;
}

}  // namespace rally

#endif  // RALLY_CORE_ALGORITHM_GEOMETRY_IMPL_POLYGON_H
