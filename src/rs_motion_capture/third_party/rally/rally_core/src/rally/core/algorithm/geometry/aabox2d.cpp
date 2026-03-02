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

#include "rally/core/algorithm/geometry/aabox2d.h"

namespace rally {

AABox2d::AABox2d(const ShortArray2f &center, const float length, const float width)
: center_(center), length_(length), width_(width), half_length_(length / 2.f),
  half_width_(width / 2.f) {
    RENSURE(length_ > -R_F_EPS);
    RENSURE(width_ > -R_F_EPS);
}

AABox2d::AABox2d(const ShortArray2f &one_corner, const ShortArray2f &opposite_corner)
: AABox2d((one_corner + opposite_corner) / 2.0, std::abs(one_corner.x - opposite_corner.x),
          std::abs(one_corner.y - opposite_corner.y)) {}

float AABox2d::distanceTo(const AABox2d &box) const {
    const float dx = std::abs(box.center_x() - center_.x) - box.half_length() - half_length_;
    const float dy = std::abs(box.center_y() - center_.y) - box.half_width() - half_width_;
    if (dx <= 0.f) {
        return std::max(0.f, dy);
    }
    if (dy <= 0.f) {
        return dx;
    }
    return std::hypot(dx, dy);
}

bool AABox2d::hasOverlap(const AABox2d &box) const {
    return std::abs(box.center_x() - center_.x) <= box.half_length() + half_length_ &&
           std::abs(box.center_y() - center_.y) <= box.half_width() + half_width_;
}

void AABox2d::shift(const ShortArray2f &shift_vec) {
    center_ += shift_vec;
}

void AABox2d::mergeFrom(const AABox2d &other_box) {
    const float x1 = std::min(min_x(), other_box.min_x());
    const float x2 = std::max(max_x(), other_box.max_x());
    const float y1 = std::min(min_y(), other_box.min_y());
    const float y2 = std::max(max_y(), other_box.max_y());
    center_ = ShortArray2f((x1 + x2) / 2.f, (y1 + y2) / 2.f);
    length_ = x2 - x1;
    width_ = y2 - y1;
    half_length_ = length_ / 2.f;
    half_width_ = width_ / 2.f;
}

std::string AABox2d::infos() const {
    std::stringstream ss;
    ss << "AABox2d (center: " << center_.infos() << ", length: " << length_ << ", width: " << width_ << ")";
    return ss.str();
}

}  // namespace rally
