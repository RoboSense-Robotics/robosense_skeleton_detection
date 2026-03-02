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
#ifndef RALLY_CORE_ALGORITHM_GEOMETRY_IMPL_LINE_SEGMENT2D_H
#define RALLY_CORE_ALGORITHM_GEOMETRY_IMPL_LINE_SEGMENT2D_H

namespace rally {

inline bool isWithin(float val, float bound1, float bound2) {
    if (bound1 > bound2) {
        std::swap(bound1, bound2);
    }
    return val >= bound1 - R_F_EPS && val <= bound2 + R_F_EPS;
}

template<typename PointT>
LineSegment2d::LineSegment2d(const PointT &start, const PointT &end) {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    start_ = ShortArray2f(start.x, start.y);
    end_ = ShortArray2f(end.x, end.y);
    const auto &dx = end_.x - start_.x;
    const auto &dy = end_.y - start_.y;
    length_ = std::hypot(dx, dy);
    unit_direction_ = (length_ <= R_F_EPS ? ShortArray2f(0.f, 0.f)
                                          : ShortArray2f(dx / length_, dy / length_));
    heading_ = getRad(unit_direction_);
}

template<typename PointT>
float LineSegment2d::distanceTo(const PointT &point) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    if (length_ <= R_F_EPS) {
        return std::hypot(point.x - start_.x, point.y - start_.y);
    }
    const float x0 = point.x - start_.x;
    const float y0 = point.y - start_.y;
    const float proj = x0 * unit_direction_.x + y0 * unit_direction_.y;
    if (proj <= 0.f) {
        return std::hypot(x0, y0);
    }
    if (proj >= length_) {
        return std::hypot(point.x - end_.x, point.y - end_.y);
    }
    return std::abs(x0 * unit_direction_.y - y0 * unit_direction_.x);
}

template<typename PointT>
float LineSegment2d::distanceTo(const PointT &point, PointT &nearest_pt) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    if (length_ <= R_F_EPS) {
        nearest_pt = start_;
        return std::hypot(point.x - start_.x, point.y - start_.y);
    }
    const float x0 = point.x - start_.x;
    const float y0 = point.y - start_.y;
    const float proj = x0 * unit_direction_.x + y0 * unit_direction_.y;
    if (proj < 0.f) {
        nearest_pt = start_;
        return hypot(x0, y0);
    }
    if (proj > length_) {
        nearest_pt = end_;
        return std::hypot(point.x - end_.x, point.y - end_.y);
    }
    const auto &tmp_pt = start_ + unit_direction_ * proj;
    nearest_pt.x = tmp_pt.x;
    nearest_pt.y = tmp_pt.y;
    return std::abs(x0 * unit_direction_.y - y0 * unit_direction_.x);
}

template<typename PointT>
float LineSegment2d::distanceSquareTo(const PointT &point) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    if (length_ <= R_F_EPS) {
        return std::hypot(point.x - start_.x, point.y - start_.y);
    }
    const float x0 = point.x - start_.x;
    const float y0 = point.y - start_.y;
    const float proj = x0 * unit_direction_.x + y0 * unit_direction_.y;
    if (proj <= 0.f) {
        return x0 * x0 + y0 * y0;
    }
    if (proj >= length_) {
        return std::hypot(point.x - end_.x, point.y - end_.y);
    }
    return (x0 * unit_direction_.y - y0 * unit_direction_.x) * (x0 * unit_direction_.y - y0 * unit_direction_.x);
}

template<typename PointT>
float LineSegment2d::distanceSquareTo(const PointT &point, PointT &nearest_pt) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    if (length_ <= R_F_EPS) {
        nearest_pt = start_;
        return std::hypot(point.x - start_.x, point.y - start_.y);
    }
    const float x0 = point.x - start_.x;
    const float y0 = point.y - start_.y;
    const float proj = x0 * unit_direction_.x + y0 * unit_direction_.y;
    if (proj <= 0.f) {
        nearest_pt = start_;
        return x0 * x0 + y0 * y0;
    }
    if (proj >= length_) {
        nearest_pt = end_;
        return std::hypot(point.x - end_.x, point.y - end_.y);
    }
    const auto &tmp_pt = start_ + unit_direction_ * proj;
    nearest_pt.x = tmp_pt.x;
    nearest_pt.y = tmp_pt.y;
    return (x0 * unit_direction_.y - y0 * unit_direction_.x) * (x0 * unit_direction_.y - y0 * unit_direction_.x);
}

template<typename PointT>
bool LineSegment2d::isPointIn(const PointT &point) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    if (length_ <= R_F_EPS) {
        return std::abs(point.x - start_.x) <= R_F_EPS &&
               std::abs(point.y - start_.y) <= R_F_EPS;
    }
    ShortArray2f tmp_pt(point.x, point.y);
    const float prod = crossProd(tmp_pt, start_, end_);
    if (std::abs(prod) > R_F_EPS) {
        return false;
    }
    return isWithin(point.x, start_.x, end_.x) &&
           isWithin(point.y, start_.y, end_.y);
}

template<typename PointT>
bool LineSegment2d::getIntersect(const LineSegment2d &other_segment, PointT &point) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    if (isPointIn(other_segment.start())) {
        point.x = other_segment.start().x;
        point.y = other_segment.start().y;
        return true;
    }
    if (isPointIn(other_segment.end())) {
        point.x = other_segment.end().x;
        point.y = other_segment.end().y;
        return true;
    }
    if (other_segment.isPointIn(start_)) {
        point.x = start_.x;
        point.y = start_.y;
        return true;
    }
    if (other_segment.isPointIn(end_)) {
        point.x = end_.x;
        point.y = end_.y;
        return true;
    }
    if (length_ <= R_F_EPS || other_segment.length() <= R_F_EPS) {
        return false;
    }
    const float cc1 = crossProd(start_, end_, other_segment.start());
    const float cc2 = crossProd(start_, end_, other_segment.end());
    if (cc1 * cc2 >= -R_F_EPS) {
        return false;
    }
    const float cc3 = crossProd(other_segment.start(), other_segment.end(), start_);
    const float cc4 = crossProd(other_segment.start(), other_segment.end(), end_);
    if (cc3 * cc4 >= -R_F_EPS) {
        return false;
    }
    const float ratio = cc4 / (cc4 - cc3);
    const auto &tmp_pt = ShortArray2f(start_.x * ratio + end_.x * (1.f - ratio),
                                      start_.y * ratio + end_.y * (1.f - ratio));
    point.x = tmp_pt.x;
    point.y = tmp_pt.y;
    return true;
}

template<typename PointT>
float LineSegment2d::projectOntoUnit(const PointT &point) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    ShortArray2f tmp(point.x - start_.x, point.y - start_.y);
    return innerProd(unit_direction_, tmp);
}

template<typename PointT>
float LineSegment2d::productOntoUnit(const PointT &point) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    ShortArray2f tmp(point.x - start_.x, point.y - start_.y);
    return crossProd(unit_direction_, tmp);
}

template<typename PointT>
float LineSegment2d::getPerpendicularFoot(const PointT &point, PointT &foot_point) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    if (length_ <= R_F_EPS) {
        foot_point.x = start_.x;
        foot_point.y = start_.y;
        return std::hypot(point.x - start_.x, point.y - start_.y);
    }
    const float x0 = point.x - start_.x;
    const float y0 = point.y - start_.y;
    const float proj = x0 * unit_direction_.x + y0 * unit_direction_.y;
    const auto &tmp = start_ + unit_direction_ * proj;
    foot_point.x = tmp.x;
    foot_point.y = tmp.y;
    return std::abs(x0 * unit_direction_.y - y0 * unit_direction_.x);
}

inline std::ostream &operator<<(std::ostream &os, const LineSegment2d &p) {
    os << p.infos();
    return os;
}

}  // namespace rally

#endif  // RALLY_CORE_ALGORITHM_GEOMETRY_IMPL_LINE_SEGMENT2D_H
