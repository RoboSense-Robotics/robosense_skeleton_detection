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

#include "rally/core/algorithm/geometry/line_segment2d.h"

namespace rally {

LineSegment2d::LineSegment2d() { unit_direction_ = ShortArray2f(1.f, 0.f); }

LineSegment2d::LineSegment2d(const ShortArray2f &start, const ShortArray2f &end)
: start_(start), end_(end) {
    const auto &dx = end_.x - start_.x;
    const auto &dy = end_.y - start_.y;
    length_ = std::hypot(dx, dy);
    unit_direction_ = (length_ <= R_F_EPS ? ShortArray2f(0.f, 0.f)
                                          : ShortArray2f(dx / length_, dy / length_));
    heading_ = getRad(unit_direction_);
}

bool LineSegment2d::isPointIn(const ShortArray2f &point) const {
    if (length_ <= R_F_EPS) {
        return std::abs(point.x - start_.x) <= R_F_EPS &&
               std::abs(point.y - start_.y) <= R_F_EPS;
    }
    const float prod = crossProd(point, start_, end_);
    if (std::abs(prod) > R_F_EPS) {
        return false;
    }
    return isWithin(point.x, start_.x, end_.x) &&
           isWithin(point.y, start_.y, end_.y);
}

ShortArray2f LineSegment2d::rotate(const float rad) {
    ShortArray2f diff_vec = end_ - start_;
    diff_vec = rotateVec(diff_vec, rad);
    return start_ + diff_vec;
}

bool LineSegment2d::hasIntersect(const LineSegment2d &other_segment) const {
    ShortArray2f point;
    return getIntersect(other_segment, point);
}

std::string LineSegment2d::infos() const {
    std::stringstream ss;
    ss << "LineSegment2d (start: " << start_.infos() << ", end: " << end_.infos() << ")";
    return ss.str();
}

}  // namespace rally
