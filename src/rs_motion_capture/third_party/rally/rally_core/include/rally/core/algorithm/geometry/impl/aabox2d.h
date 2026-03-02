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
#ifndef RALLY_CORE_ALGORITHM_GEOMETRY_IMPL_AABOX2D_H
#define RALLY_CORE_ALGORITHM_GEOMETRY_IMPL_AABOX2D_H

namespace rally {

template<typename PointT>
AABox2d::AABox2d(const std::vector<PointT> &points) {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    RENSURE(!points.empty());
    float min_x = points[0].x;
    float max_x = points[0].x;
    float min_y = points[0].y;
    float max_y = points[0].y;
    for (const auto &point : points) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }

    center_ = {(min_x + max_x) / 2.f, (min_y + max_y) / 2.f};
    length_ = max_x - min_x;
    width_ = max_y - min_y;
    half_length_ = length_ / 2.f;
    half_width_ = width_ / 2.f;
}

template<typename PointT>
void AABox2d::getAllCorners(std::vector<PointT> &corners) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    corners.clear();
    corners.reserve(4);
    corners.emplace_back(center_.x + half_length_, center_.y - half_width_);
    corners.emplace_back(center_.x + half_length_, center_.y + half_width_);
    corners.emplace_back(center_.x - half_length_, center_.y + half_width_);
    corners.emplace_back(center_.x - half_length_, center_.y - half_width_);
}

template<typename PointT>
bool AABox2d::isPointIn(const PointT &point) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    return std::abs(point.x - center_.x) <= half_length_ + R_F_EPS &&
           std::abs(point.y - center_.y) <= half_width_ + R_F_EPS;
}

template<typename PointT>
bool AABox2d::isPointOnBoundary(const PointT &point) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    const float dx = std::abs(point.x - center_.x);
    const float dy = std::abs(point.y - center_.y);
    return (std::abs(dx - half_length_) <= R_F_EPS && dy <= half_width_ + R_F_EPS) ||
           (std::abs(dy - half_width_) <= R_F_EPS && dx <= half_length_ + R_F_EPS);
}

template<typename PointT>
float AABox2d::distanceTo(const PointT &point) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    const float dx = std::abs(point.x - center_.x) - half_length_;
    const float dy = std::abs(point.y - center_.y) - half_width_;
    if (dx <= 0.f) {
        return std::max(0.f, dy);
    }
    if (dy <= 0.f) {
        return dx;
    }
    return std::hypot(dx, dy);
}

template<typename PointT>
void AABox2d::mergeFrom(const PointT &other_point) {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    const float x1 = std::min(min_x(), other_point.x);
    const float x2 = std::max(max_x(), other_point.x);
    const float y1 = std::min(min_y(), other_point.y);
    const float y2 = std::max(max_y(), other_point.y);
    center_ = ShortArray2f((x1 + x2) / 2.f, (y1 + y2) / 2.f);
    length_ = x2 - x1;
    width_ = y2 - y1;
    half_length_ = length_ / 2.f;
    half_width_ = width_ / 2.f;
}

inline std::ostream &operator<<(std::ostream &os, const AABox2d &p) {
    os << p.infos();
    return os;
}

}  // namespace rally

#endif  // RALLY_CORE_ALGORITHM_GEOMETRY_IMPL_AABOX2D_H
