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

#include "rally/core/algorithm/geometry/geo_utils.h"
#include "rally/core/algorithm/geometry/polygon2d.h"

namespace rally {

Polygon2d::Polygon2d(const Box2d &box) {
    points_ = box.getAllCorners();
    buildFromPoints();
}

Polygon2d::Polygon2d(std::vector<ShortArray2f> points) {
    points_ = points;
    buildFromPoints();
}

void Polygon2d::buildFromPoints() {
    num_points_ = static_cast<uint32_t>(points_.size());
    RENSURE(num_points_ >= 3);

    // Make sure the points are in ccw order.
    area_ = 0.f;
    for (uint32_t i{1}; i < num_points_; ++i) {
        area_ += crossProd(points_[0], points_[i - 1], points_[i]);
    }
    if (area_ < 0.f) {
        area_ = -area_;
        std::reverse(points_.begin(), points_.end());
    }
    area_ /= 2.f;
    RENSURE(area_ > R_F_EPS);

    // Construct line_segments.
    line_segments_.reserve(num_points_);
    for (uint32_t i = 0; i < num_points_; ++i) {
        line_segments_.emplace_back(points_[i], points_[next(i)]);
    }

    // Check convexity.
    is_convex_ = true;
    for (uint32_t i{0}; i < num_points_; ++i) {
        if (crossProd(points_[prev(i)], points_[i], points_[next(i)]) <= -R_F_EPS) {
            is_convex_ = false;
            break;
        }
    }

    // Compute aabox.
    min_x_ = points_[0].x;
    max_x_ = points_[0].x;
    min_y_ = points_[0].y;
    max_y_ = points_[0].y;
    for (const auto &point : points_) {
        min_x_ = std::min(min_x_, point.x);
        max_x_ = std::max(max_x_, point.x);
        min_y_ = std::min(min_y_, point.y);
        max_y_ = std::max(max_y_, point.y);
    }
}

float Polygon2d::distanceTo(const LineSegment2d &line_segment) const {
    if (line_segment.length() <= R_F_EPS) {
        return distanceTo(line_segment.start());
    }
    RENSURE(points_.size() >= 3U);
    if (isPointIn(line_segment.center())) {
        return 0.f;
    }
    if (std::any_of(line_segments_.begin(), line_segments_.end(),
                    [&](const LineSegment2d &poly_seg) {
                        return poly_seg.hasIntersect(line_segment);
                    })) {
        return 0.f;
    }

    float distance = std::min(distanceTo(line_segment.start()),
                              distanceTo(line_segment.end()));
    for (uint32_t i{0}; i < num_points_; ++i) {
        distance = std::min(distance, line_segment.distanceTo(points_[i]));
    }
    return distance;
}

float Polygon2d::distanceTo(const Box2d &box) const {
    RENSURE(points_.size() >= 3U);
    return distanceTo(Polygon2d(box));
}

float Polygon2d::distanceTo(const Polygon2d &polygon) const {
    RENSURE(points_.size() >= 3U);
    RENSURE(polygon.num_points() >= 3);

    if (isPointIn(polygon.points()[0])) {
        return 0.f;
    }
    if (polygon.isPointIn(points_[0])) {
        return 0.f;
    }
    float distance = std::numeric_limits<float>::infinity();
    for (uint32_t i{0}; i < num_points_; ++i) {
        distance = std::min(distance, polygon.distanceTo(line_segments_[i]));
    }
    return distance;
}

bool Polygon2d::contains(const LineSegment2d &line_segment) const {
    if (line_segment.length() <= R_F_EPS) {
        return isPointIn(line_segment.start());
    }
    RENSURE(points_.size() >= 3U);
    if (!isPointIn(line_segment.start())) {
        return false;
    }
    if (!isPointIn(line_segment.end())) {
        return false;
    }
    if (!is_convex_) {
        std::vector<LineSegment2d> overlaps = getAllOverlaps(line_segment);
        float total_length = 0.f;
        for (const auto &overlap_seg : overlaps) {
            total_length += overlap_seg.length();
        }
        return total_length >= line_segment.length() - R_F_EPS;
    }
    return true;
}

bool Polygon2d::contains(const Polygon2d &polygon) const {
    RENSURE(points_.size() >= 3U);
    if (area_ < polygon.area() - R_F_EPS) {
        return false;
    }
    if (!isPointIn(polygon.points()[0])) {
        return false;
    }
    const auto &line_segments = polygon.line_segments();
    return std::all_of(line_segments.begin(), line_segments.end(),
                       [&](const LineSegment2d &line_segment) {
                           return contains(line_segment);
                       });
}

bool Polygon2d::hasOverlap(const LineSegment2d &line_segment) const {
    RENSURE(points_.size() >= 3U);
    if ((line_segment.start().x < min_x_ && line_segment.end().x < min_x_) ||
        (line_segment.start().x > max_x_ && line_segment.end().x > max_x_) ||
        (line_segment.start().y < min_y_ && line_segment.end().y < min_y_) ||
        (line_segment.start().y > max_y_ && line_segment.end().y > max_y_)) {
        return false;
    }
    ShortArray2f first;
    ShortArray2f last;
    return getOverlap(line_segment, first, last);
}

bool Polygon2d::getOverlap(const LineSegment2d &line_segment, ShortArray2f &first, ShortArray2f &last) const {
    RENSURE(points_.size() >= 3U);

    if (line_segment.length() <= R_F_EPS) {
        if (!isPointIn(line_segment.start())) {
            return false;
        }
        first = line_segment.start();
        last = line_segment.start();
        return true;
    }

    float min_proj = line_segment.length();
    float max_proj = 0.f;
    if (isPointIn(line_segment.start())) {
        first = line_segment.start();
        min_proj = 0.f;
    }
    if (isPointIn(line_segment.end())) {
        last = line_segment.end();
        max_proj = line_segment.length();
    }
    for (const auto &poly_seg : line_segments_) {
        ShortArray2f pt;
        if (poly_seg.getIntersect(line_segment, pt)) {
            const float proj = line_segment.projectOntoUnit(pt);
            if (proj < min_proj) {
                min_proj = proj;
                first = pt;
            }
            if (proj > max_proj) {
                max_proj = proj;
                last = pt;
            }
        }
    }
    return min_proj <= max_proj + R_F_EPS;
}

std::vector<LineSegment2d> Polygon2d::getAllOverlaps(const LineSegment2d &line_segment) const {
    RENSURE(points_.size() >= 3U);

    if (line_segment.length() <= R_F_EPS) {
        std::vector<LineSegment2d> overlaps;
        if (isPointIn(line_segment.start())) {
            overlaps.push_back(line_segment);
        }
        return overlaps;
    }
    std::vector<float> projections;
    if (isPointIn(line_segment.start())) {
        projections.push_back(0.f);
    }
    if (isPointIn(line_segment.end())) {
        projections.push_back(line_segment.length());
    }
    for (const auto &poly_seg : line_segments_) {
        ShortArray2f pt;
        if (poly_seg.getIntersect(line_segment, pt)) {
            projections.push_back(line_segment.projectOntoUnit(pt));
        }
    }
    std::sort(projections.begin(), projections.end());
    std::vector<std::pair<float, float> > overlaps;
    for (size_t i{0}; i + 1 < projections.size(); ++i) {
        const float start_proj = projections[i];
        const float end_proj = projections[i + 1];
        if (end_proj - start_proj <= R_F_EPS) {
            continue;
        }
        const ShortArray2f reference_point = line_segment.start() +
                                             (start_proj + end_proj) / 2.f * line_segment.unit_direction();
        if (!isPointIn(reference_point)) {
            continue;
        }
        if (overlaps.empty() || start_proj > overlaps.back().second + R_F_EPS) {
            overlaps.emplace_back(start_proj, end_proj);
        } else {
            overlaps.back().second = end_proj;
        }
    }
    std::vector<LineSegment2d> overlap_line_segments;
    for (const auto &overlap : overlaps) {
        overlap_line_segments.emplace_back(
        line_segment.start() + overlap.first * line_segment.unit_direction(),
        line_segment.start() + overlap.second * line_segment.unit_direction());
    }
    return overlap_line_segments;
}

bool Polygon2d::hasOverlap(const Polygon2d &polygon) const {
    RENSURE(points_.size() >= 3U);
    if (polygon.max_x() < min_x() || polygon.min_x() > max_x() ||
        polygon.max_y() < min_y() || polygon.min_y() > max_y()) {
        return false;
    }
    return distanceTo(polygon) <= R_F_EPS;
}

bool Polygon2d::computeOverlap(const Polygon2d &other_polygon, Polygon2d &overlap_polygon) const {
    RENSURE(points_.size() >= 3U);
    RENSURE(is_convex_ && other_polygon.is_convex());
    std::vector<ShortArray2f> points = other_polygon.points();
    for (uint32_t i = 0; i < num_points_; ++i) {
        if (!clipConvexHull(line_segments_[i], points)) {
            return false;
        }
    }
    return computeConvexHull(points, overlap_polygon);
}

float Polygon2d::computeIoU(const Polygon2d &other_polygon) const {
    Polygon2d overlap_polygon;
    if (!computeOverlap(other_polygon, overlap_polygon)) {
        return 0.f;
    }
    float intersection_area = overlap_polygon.area();
    float union_area = area_ + other_polygon.area() - overlap_polygon.area();
    return intersection_area / union_area;
}

AABox2d Polygon2d::getAABoundingBox() const {
    return AABox2d({min_x_, min_y_}, {max_x_, max_y_});
}

void Polygon2d::extremePoints(const float heading, ShortArray2f &first, ShortArray2f &last) const {
    RENSURE(points_.size() >= 3U);

    const auto &direction_vec = createUnitVec2d(heading);
    float min_proj = std::numeric_limits<float>::infinity();
    float max_proj = -std::numeric_limits<float>::infinity();
    for (const auto &pt : points_) {
        const auto &proj = innerProd(pt, direction_vec);
        if (proj < min_proj) {
            min_proj = proj;
            first = pt;
        }
        if (proj > max_proj) {
            max_proj = proj;
            last = pt;
        }
    }
}

Polygon2d Polygon2d::expandByDistance(const float distance) const {
    if (!is_convex_) {
        Polygon2d convex_polygon;
        computeConvexHull(points_, convex_polygon);
        RENSURE(convex_polygon.is_convex());
        return convex_polygon.expandByDistance(distance);
    }
    const float kMinAngle = 0.1f;
    std::vector<ShortArray2f> points;
    for (uint32_t i{0}; i < num_points_; ++i) {
        const float start_angle = line_segments_[prev(i)].heading() - R_M_HALF_PI;
        const float end_angle = line_segments_[i].heading() - R_M_HALF_PI;
        const float diff = wrapAngle(end_angle - start_angle);
        if (diff <= R_F_EPS) {
            points.push_back(points_[i] + createUnitVec2d(start_angle) * distance);
        } else {
            const uint32_t &count = static_cast<uint32_t>(diff / kMinAngle) + 1;
            for (uint32_t k = 0; k <= count; ++k) {
                const float angle = start_angle + diff * static_cast<float>(k) / static_cast<float>(count);
                points.push_back(points_[i] + createUnitVec2d(angle) * distance);
            }
        }
    }
    Polygon2d new_polygon;
    RENSURE(computeConvexHull(points, new_polygon));
    return new_polygon;
}

Box2d Polygon2d::getBoundingBoxWithHeading(const float heading) const {
    RENSURE(points_.size() >= 3U);
    const auto &direction_vec = createUnitVec2d(heading);
    ShortArray2f px1;
    ShortArray2f px2;
    ShortArray2f py1;
    ShortArray2f py2;
    extremePoints(heading, px1, px2);
    extremePoints(heading - R_M_HALF_PI, py1, py2);
    const float x1 = innerProd(px1, direction_vec);
    const float x2 = innerProd(px2, direction_vec);
    const float y1 = crossProd(py1, direction_vec);
    const float y2 = crossProd(py2, direction_vec);
    return Box2d((x1 + x2) / 2.f * direction_vec + (y1 + y2) / 2.f * ShortArray2f(direction_vec.y, -direction_vec.x),
                 heading, x2 - x1, y2 - y1);
}

Box2d Polygon2d::getMinAreaBoundingBox() const {
    RENSURE(points_.size() >= 3U);
    if (!is_convex_) {
        Polygon2d convex_polygon;
        computeConvexHull(points_, convex_polygon);
        RENSURE(convex_polygon.is_convex());
        return convex_polygon.getMinAreaBoundingBox();
    }

    float min_area = std::numeric_limits<float>::infinity();
    float min_area_at_heading{0.f};
    uint32_t left_most{0};
    uint32_t right_most{0};
    uint32_t top_most{0};
    for (uint32_t i{0}; i < num_points_; ++i) {
        const auto &line_segment = line_segments_[i];
        float proj{0.f};
        float min_proj = line_segment.projectOntoUnit(points_[left_most]);
        while ((proj = line_segment.projectOntoUnit(points_[prev(left_most)])) < min_proj) {
            min_proj = proj;
            left_most = prev(left_most);
        }
        while ((proj = line_segment.projectOntoUnit(points_[next(left_most)])) < min_proj) {
            min_proj = proj;
            left_most = next(left_most);
        }
        float max_proj = line_segment.projectOntoUnit(points_[right_most]);
        while ((proj = line_segment.projectOntoUnit(points_[prev(right_most)])) > max_proj) {
            max_proj = proj;
            right_most = prev(right_most);
        }
        while ((proj = line_segment.projectOntoUnit(points_[next(right_most)])) > max_proj) {
            max_proj = proj;
            right_most = next(right_most);
        }
        float prod{0.f};
        float max_prod = line_segment.productOntoUnit(points_[top_most]);
        while ((prod = line_segment.productOntoUnit(points_[prev(top_most)])) > max_prod) {
            max_prod = prod;
            top_most = prev(top_most);
        }
        while ((prod = line_segment.productOntoUnit(points_[next(top_most)])) > max_prod) {
            max_prod = prod;
            top_most = next(top_most);
        }
        const float area = max_prod * (max_proj - min_proj);
        if (area < min_area) {
            min_area = area;
            min_area_at_heading = line_segment.heading();
        }
    }
    return getBoundingBoxWithHeading(min_area_at_heading);
}

bool Polygon2d::clipConvexHull(const LineSegment2d &line_segment, std::vector<ShortArray2f> &points) {
    if (line_segment.length() <= R_F_EPS) {
        return true;
    }
    const size_t n = points.size();
    if (n < 3) {
        return false;
    }
    std::vector<float> prod(n);
    std::vector<int32_t> side(n);
    for (size_t i = 0; i < n; ++i) {
        prod[i] = crossProd(line_segment.start(), line_segment.end(), points[i]);
        if (std::abs(prod[i]) <= R_F_EPS) {
            side[i] = 0;
        } else {
            side[i] = ((prod[i] < 0) ? -1 : 1);
        }
    }

    std::vector<ShortArray2f> new_points;
    for (size_t i = 0; i < n; ++i) {
        if (side[i] >= 0) {
            new_points.push_back(points[i]);
        }
        const size_t j = ((i == n - 1) ? 0 : (i + 1));
        if (side[i] * side[j] < 0) {
            const float ratio = prod[j] / (prod[j] - prod[i]);
            new_points.emplace_back(
            points[i].x * ratio + points[j].x * (1.f - ratio),
            points[i].y * ratio + points[j].y * (1.f - ratio));
        }
    }

    points.swap(new_points);
    return points.size() >= 3U;
}

std::string Polygon2d::infos() const {
    std::stringstream ss;
    ss << "Polygon2d (num_points: " << num_points_;
    uint32_t step = num_points_ / 10;
    for (uint32_t i = 0; i < num_points_; i += step) {
        ss << ", " << i << "th: " << points_[i].infos();
    }
    ss << ", " << (is_convex_ ? "convex" : "non-convex") << ", area: " << area_ << ")";
    return ss.str();
}

}  // namespace rally
