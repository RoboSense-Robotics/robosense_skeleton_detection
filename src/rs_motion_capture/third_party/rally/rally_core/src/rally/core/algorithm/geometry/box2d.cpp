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
#include "rally/core/algorithm/geometry/box2d.h"

namespace rally {

namespace {

float ptToSegDistance(float query_x, float query_y, float start_x,
                      float start_y, float end_x, float end_y,
                      float length) {
    const float x0 = query_x - start_x;
    const float y0 = query_y - start_y;
    const float dx = end_x - start_x;
    const float dy = end_y - start_y;
    const float proj = x0 * dx + y0 * dy;
    if (proj <= 0.f) {
        return std::hypot(x0, y0);
    }
    if (proj >= length * length) {
        return hypot(x0 - dx, y0 - dy);
    }
    return std::abs(x0 * dy - y0 * dx) / length;
}

}  // namespace

Box2d::Box2d(const ShortArray2f &center, const float heading, const float length, const float width)
: center_{center}, length_{length}, width_{width}, half_length_(length / 2.f),
  half_width_(width / 2.f), heading_(heading), cos_heading_(std::cos(heading)),
  sin_heading_(std::sin(heading)) {
    RENSURE(length_ > -R_F_EPS);
    RENSURE(width_ > -R_F_EPS);
    initCorners();
}

Box2d::Box2d(const LineSegment2d &axis, const float width)
: center_(axis.center()), length_(axis.length()), width_(width), half_length_(axis.length() / 2.f),
  half_width_(width / 2.f), heading_(axis.heading()), cos_heading_(axis.cos_heading()),
  sin_heading_(axis.sin_heading()) {
    RENSURE(length_ > -R_F_EPS);
    RENSURE(width_ > -R_F_EPS);
    initCorners();
}

void Box2d::initCorners() {
    const float dx1 = cos_heading_ * half_length_;
    const float dy1 = sin_heading_ * half_length_;
    const float dx2 = sin_heading_ * half_width_;
    const float dy2 = -cos_heading_ * half_width_;
    corners_.clear();
    corners_.emplace_back(center_.x + dx1 + dx2, center_.y + dy1 + dy2);
    corners_.emplace_back(center_.x + dx1 - dx2, center_.y + dy1 - dy2);
    corners_.emplace_back(center_.x - dx1 - dx2, center_.y - dy1 - dy2);
    corners_.emplace_back(center_.x - dx1 + dx2, center_.y - dy1 + dy2);

    for (auto &corner : corners_) {
        max_x_ = std::fmax(corner.x, max_x_);
        min_x_ = std::fmin(corner.x, min_x_);
        max_y_ = std::fmax(corner.y, max_y_);
        min_y_ = std::fmin(corner.y, min_y_);
    }
}

Box2d::Box2d(const AABox2d &aabox)
: center_(aabox.center()), length_(aabox.length()), width_(aabox.width()),
  half_length_(aabox.half_length()), half_width_(aabox.half_width()),
  heading_(0.f), cos_heading_(1.f), sin_heading_(0.f) {
    RENSURE(length_ > -R_F_EPS);
    RENSURE(width_ > -R_F_EPS);
}

float Box2d::distanceTo(const LineSegment2d &line_segment) const {
    if (line_segment.length() <= R_F_EPS) {
        return distanceTo(line_segment.start());
    }
    const float ref_x1 = line_segment.start().x - center_.x;
    const float ref_y1 = line_segment.start().y - center_.y;
    float x1 = ref_x1 * cos_heading_ + ref_y1 * sin_heading_;
    float y1 = ref_x1 * sin_heading_ - ref_y1 * cos_heading_;
    float box_x = half_length_;
    float box_y = half_width_;
    int32_t gx1 = (x1 >= box_x ? 1 : (x1 <= -box_x ? -1 : 0));
    int32_t gy1 = (y1 >= box_y ? 1 : (y1 <= -box_y ? -1 : 0));
    if (gx1 == 0 && gy1 == 0) {
        return 0.f;
    }
    const float ref_x2 = line_segment.end().x - center_.x;
    const float ref_y2 = line_segment.end().y - center_.y;
    float x2 = ref_x2 * cos_heading_ + ref_y2 * sin_heading_;
    float y2 = ref_x2 * sin_heading_ - ref_y2 * cos_heading_;
    int32_t gx2 = (x2 >= box_x ? 1 : (x2 <= -box_x ? -1 : 0));
    int32_t gy2 = (y2 >= box_y ? 1 : (y2 <= -box_y ? -1 : 0));
    if (gx2 == 0 && gy2 == 0) {
        return 0.f;
    }
    if (gx1 < 0 || (gx1 == 0 && gx2 < 0)) {
        x1 = -x1;
        gx1 = -gx1;
        x2 = -x2;
        gx2 = -gx2;
    }
    if (gy1 < 0 || (gy1 == 0 && gy2 < 0)) {
        y1 = -y1;
        gy1 = -gy1;
        y2 = -y2;
        gy2 = -gy2;
    }
    if (gx1 < gy1 || (gx1 == gy1 && gx2 < gy2)) {
        std::swap(x1, y1);
        std::swap(gx1, gy1);
        std::swap(x2, y2);
        std::swap(gx2, gy2);
        std::swap(box_x, box_y);
    }
    if (gx1 == 1 && gy1 == 1) {
        switch (gx2 * 3 + gy2) {
            case 4:
                return ptToSegDistance(box_x, box_y, x1, y1, x2, y2, line_segment.length());
            case 3:
                return (x1 > x2) ? (x2 - box_x)
                                 : ptToSegDistance(box_x, box_y, x1, y1, x2, y2, line_segment.length());
            case 2:
                return (x1 > x2) ? ptToSegDistance(box_x, -box_y, x1, y1, x2, y2, line_segment.length())
                                 : ptToSegDistance(box_x, box_y, x1, y1, x2, y2, line_segment.length());
            case -1:
                return crossProd({x1, y1}, {x2, y2}, {box_x, -box_y}) >= 0.f
                       ? 0.f
                       : ptToSegDistance(box_x, -box_y, x1, y1, x2, y2, line_segment.length());
            case -4:
                return crossProd({x1, y1}, {x2, y2}, {box_x, -box_y}) <= 0.f
                       ? ptToSegDistance(box_x, -box_y, x1, y1, x2, y2, line_segment.length())
                       : (crossProd({x1, y1}, {x2, y2}, {-box_x, box_y}) <= 0.f
                          ? 0.f
                          : ptToSegDistance(-box_x, box_y, x1, y1, x2, y2, line_segment.length()));
        }
    } else {
        switch (gx2 * 3 + gy2) {
            case 4:
                return (x1 < x2) ? (x1 - box_x)
                                 : ptToSegDistance(box_x, box_y, x1, y1, x2, y2, line_segment.length());
            case 3:
                return std::min(x1, x2) - box_x;
            case 1:
            case -2:
                return crossProd({x1, y1}, {x2, y2}, {box_x, box_y}) <= 0.f
                       ? 0.f
                       : ptToSegDistance(box_x, box_y, x1, y1, x2, y2, line_segment.length());
            case -3:
                return 0.f;
        }
    }
    std::stringstream ss;
    ss << "Box2d: unimplemented state: " << gx1 << " " << gy1 << " " << gx2 << " " << gy2;
    RTHROW(ss.str());
}

float Box2d::distanceTo(const Box2d &box) const {
    return Polygon2d(box).distanceTo(*this);
}

bool Box2d::hasOverlap(const LineSegment2d &line_segment) const {
    if (line_segment.length() <= R_F_EPS) {
        return isPointIn(line_segment.start());
    }
    if (std::fmax(line_segment.start().x, line_segment.end().x) < min_x() ||
        std::fmin(line_segment.start().x, line_segment.end().x) > max_x() ||
        std::fmax(line_segment.start().y, line_segment.end().y) < min_y() ||
        std::fmin(line_segment.start().y, line_segment.end().y) > max_y()) {
        return false;
    }
    return distanceTo(line_segment) <= R_F_EPS;
}

bool Box2d::hasOverlap(const Box2d &box) const {
    if (box.max_x() < min_x() || box.min_x() > max_x() || box.max_y() < min_y() ||
        box.min_y() > max_y()) {
        return false;
    }

    const float shift_x = box.center_x() - center_.x;
    const float shift_y = box.center_y() - center_.y;

    const float dx1 = cos_heading_ * half_length_;
    const float dy1 = sin_heading_ * half_length_;
    const float dx2 = sin_heading_ * half_width_;
    const float dy2 = -cos_heading_ * half_width_;
    const float dx3 = box.cos_heading() * box.half_length();
    const float dy3 = box.sin_heading() * box.half_length();
    const float dx4 = box.sin_heading() * box.half_width();
    const float dy4 = -box.cos_heading() * box.half_width();

    return std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
           std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) +
           std::abs(dx4 * cos_heading_ + dy4 * sin_heading_) + half_length_ &&
           std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
           std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) +
           std::abs(dx4 * sin_heading_ - dy4 * cos_heading_) + half_width_ &&
           std::abs(shift_x * box.cos_heading() + shift_y * box.sin_heading()) <=
           std::abs(dx1 * box.cos_heading() + dy1 * box.sin_heading()) +
           std::abs(dx2 * box.cos_heading() + dy2 * box.sin_heading()) + box.half_length() &&
           std::abs(shift_x * box.sin_heading() - shift_y * box.cos_heading()) <=
           std::abs(dx1 * box.sin_heading() - dy1 * box.cos_heading()) +
           std::abs(dx2 * box.sin_heading() - dy2 * box.cos_heading()) +
           box.half_width();
}

float Box2d::getOverlapArea(const Box2d &box) const {
    std::vector<ShortArray2f> inter;
    auto res = getInterSection(*this, box, inter);
    if (inter.empty() || res == RectanglesIntersectTypes::INTERSECT_NONE) {
        return 0.f;
    }
    if (res == RectanglesIntersectTypes::INTERSECT_FULL) {
        return std::min(area(), box.area());
    }
    return contourArea(inter);
}

AABox2d Box2d::getAABox() const {
    const float dx1 = std::abs(cos_heading_ * half_length_);
    const float dy1 = std::abs(sin_heading_ * half_length_);
    const float dx2 = std::abs(sin_heading_ * half_width_);
    const float dy2 = std::abs(cos_heading_ * half_width_);
    return AABox2d(center_, (dx1 + dx2) * 2.f, (dy1 + dy2) * 2.f);
}

void Box2d::rotateFromCenter(const float rotate_angle) {
    heading_ = normalizeAngle(heading_ + rotate_angle);
    cos_heading_ = std::cos(heading_);
    sin_heading_ = std::sin(heading_);
    initCorners();
}

void Box2d::shift(const ShortArray2f &shift_vec) {
    center_ += shift_vec;
    initCorners();
}

void Box2d::longitudinalExtend(const float extension_length) {
    length_ += extension_length;
    half_length_ += extension_length / 2.f;
    initCorners();
}

void Box2d::lateralExtend(const float extension_length) {
    width_ += extension_length;
    half_width_ += extension_length / 2.f;
    initCorners();
}

std::string Box2d::infos() const {
    std::stringstream ss;
    ss << "Box2D (center: " << center_.infos() << ", heading: " << heading_ << ", length: " << length_
       << ", width: " << width_ << ")";
    return ss.str();
}

}  // namespace rally
