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
#ifndef RALLY_CORE_ALGORITHM_GEOMETRY_IMPL_BOX2D_H
#define RALLY_CORE_ALGORITHM_GEOMETRY_IMPL_BOX2D_H

namespace rally {

template<typename PointT>
Box2d Box2d::createAABox(const PointT &one_corner, const PointT &opposite_corner) {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    const float x1 = std::min(one_corner.x, opposite_corner.x);
    const float x2 = std::max(one_corner.x, opposite_corner.x);
    const float y1 = std::min(one_corner.y, opposite_corner.y);
    const float y2 = std::max(one_corner.y, opposite_corner.y);
    return Box2d({(x1 + x2) / 2.f, (y1 + y2) / 2.f}, 0.f, x2 - x1, y2 - y1);
}

template<typename PointT>
void Box2d::getAllCorners(std::vector<PointT> &corners) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    corners.resize(corners_.size());
    for (size_t i = 0; i < corners_.size(); ++i) {
        corners[i].x = corners_[i].x;
        corners[i].y = corners_[i].y;
    }
}

template<typename PointT>
bool Box2d::isPointIn(const PointT &point) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    const float x0 = point.x - center_.x;
    const float y0 = point.y - center_.y;
    const float dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
    const float dy = std::abs(-x0 * sin_heading_ + y0 * cos_heading_);
    return dx <= half_length_ + R_F_EPS && dy <= half_width_ + R_F_EPS;
}

template<typename PointT>
bool Box2d::isPointOnBoundary(const PointT &point) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    const float x0 = point.x - center_.x;
    const float y0 = point.y - center_.y;
    const float dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
    const float dy = std::abs(x0 * sin_heading_ - y0 * cos_heading_);
    return (std::abs(dx - half_length_) <= R_F_EPS && dy <= half_width_ + R_F_EPS) ||
           (std::abs(dy - half_width_) <= R_F_EPS && dx <= half_length_ + R_F_EPS);
}

template<typename PointT>
float Box2d::distanceTo(const PointT &point) const {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    const float x0 = point.x - center_.x;
    const float y0 = point.y - center_.y;
    const float dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_) - half_length_;
    const float dy = std::abs(x0 * sin_heading_ - y0 * cos_heading_) - half_width_;
    if (dx <= 0.f) {
        return std::max(0.f, dy);
    }
    if (dy <= 0.f) {
        return dx;
    }
    return std::hypot(dx, dy);
}

inline std::ostream &operator<<(std::ostream &os, const Box2d &p) {
    os << p.infos();
    return os;
}

template<typename PointT>
inline RectanglesIntersectTypes getInterSection(const Box2d &box1, const Box2d &box2,
                                                std::vector<PointT> &contours) {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value &&rally::details::has_y<PointT>::value);
    float area1 = box1.area();
    float area2 = box2.area();

    // L2 metric
    const float samePointEps = std::max(1.e-16f, 1.e-6f * std::max(area1, area2));

    std::vector<PointT> vec1(4), vec2(4);
    std::vector<PointT> pts1(4), pts2(4);

    std::vector<PointT> intersection;
    intersection.reserve(24);

    box1.getAllCorners(pts1);
    box2.getAllCorners(pts2);

    RectanglesIntersectTypes ret{RectanglesIntersectTypes::INTERSECT_FULL};

    // Specical case of rect1 == rect2
    {
        bool same = true;
        for (uint32_t i = 0; i < 4; i++) {
            if (std::abs(pts1[i].x - pts2[i].x) > samePointEps || (std::abs(pts1[i].y - pts2[i].y) > samePointEps)) {
                same = false;
                break;
            }
        }

        if (same) {
            intersection.resize(4);
            for (uint32_t i = 0; i < 4; i++) {
                intersection[i] = pts1[i];
            }
            contours = intersection;

            return RectanglesIntersectTypes::INTERSECT_FULL;
        }
    }

    // Line vector
    // A line from p1 to p2 is: p1 + (p2-p1)*t, t=[0,1]
    for (uint32_t i = 0; i < 4; i++) {
        vec1[i].x = pts1[(i + 1) % 4].x - pts1[i].x;
        vec1[i].y = pts1[(i + 1) % 4].y - pts1[i].y;

        vec2[i].x = pts2[(i + 1) % 4].x - pts2[i].x;
        vec2[i].y = pts2[(i + 1) % 4].y - pts2[i].y;
    }

    // Line test - test all line combos for intersection
    for (uint32_t i = 0; i < 4; i++) {
        for (uint32_t j = 0; j < 4; j++) {
            // Solve for 2x2 Ax=b
            float x21 = pts2[j].x - pts1[i].x;
            float y21 = pts2[j].y - pts1[i].y;

            float vx1 = vec1[i].x;
            float vy1 = vec1[i].y;

            float vx2 = vec2[j].x;
            float vy2 = vec2[j].y;

            float det = vx2 * vy1 - vx1 * vy2;

            float t1 = (vx2 * y21 - vy2 * x21) / det;
            float t2 = (vx1 * y21 - vy1 * x21) / det;

            // This takes care of parallel lines
            if (std::isinf(t1) || std::isinf(t2) || std::isnan(t1) || std::isnan(t2)) {
                continue;
            }

            if (t1 >= 0.0f && t1 <= 1.0f && t2 >= 0.0f && t2 <= 1.0f) {
                float xi = pts1[i].x + vec1[i].x * t1;
                float yi = pts1[i].y + vec1[i].y * t1;

                PointT t;
                t.x = xi;
                t.y = yi;
                intersection.emplace_back(t);
            }
        }
    }

    if (!intersection.empty()) {
        ret = RectanglesIntersectTypes::INTERSECT_PARTIAL;
    }

    // Check for vertices from rect1 inside recct2
    for (uint32_t i = 0; i < 4; i++) {
        // We do a sign test to see which side the point lies.
        // If the point all lie on the same sign for all 4 sides of the rect,
        // then there's an intersection
        uint32_t posSign = 0;
        uint32_t negSign = 0;

        float x = pts1[i].x;
        float y = pts1[i].y;

        for (uint32_t j = 0; j < 4; j++) {
            // line equation: Ax + By + C = 0
            // see which side of the line this point is at
            float A = -vec2[j].y;
            float B = vec2[j].x;
            float C = -(A * pts2[j].x + B * pts2[j].y);

            float s = A * x + B * y + C;

            if (s >= 0) {
                posSign++;
            } else {
                negSign++;
            }
        }

        if (posSign == 4 || negSign == 4) {
            intersection.push_back(pts1[i]);
        }
    }

    // Reverse the check - check for vertices from rect2 inside recct1
    for (uint32_t i = 0; i < 4; i++) {
        // We do a sign test to see which side the point lies.
        // If the point all lie on the same sign for all 4 sides of the rect,
        // then there's an intersection
        uint32_t posSign = 0;
        uint32_t negSign = 0;

        float x = pts2[i].x;
        float y = pts2[i].y;

        for (uint32_t j = 0; j < 4; j++) {
            // line equation: Ax + By + C = 0
            // see which side of the line this point is at
            float A = -vec1[j].y;
            float B = vec1[j].x;
            float C = -(A * pts1[j].x + B * pts1[j].y);

            float s = A * x + B * y + C;

            if (s >= 0) {
                posSign++;
            } else {
                negSign++;
            }
        }

        if (posSign == 4 || negSign == 4) {
            intersection.push_back(pts2[i]);
        }
    }

    size_t N = intersection.size();
    if (N == 0) {
        return RectanglesIntersectTypes::INTERSECT_NONE;
    }

    // Get rid of duplicated points
    size_t Nstride = N;
    std::vector<float> distPt(N * N);
    std::vector<uint32_t> ptDistRemap(N);
    for (uint32_t i = 0; i < N; ++i) {
        const auto &pt0 = intersection[i];
        ptDistRemap[i] = i;
        for (uint32_t j = i + 1; j < N;) {
            const auto &pt1 = intersection[j];
            float d2 = (pt1.x - pt0.x) * (pt1.x - pt0.x) + (pt1.y - pt0.y) * (pt1.y - pt0.y);
            if (d2 <= samePointEps) {
                if (j < N - 1) {
                    intersection[j] = intersection[N - 1];
                }
                N--;
                continue;
            }
            distPt[i * Nstride + j] = d2;
            ++j;
        }
    }
    while (N > 8) {  // we still have duplicate points after samePointEps threshold (eliminate closest points)
        uint32_t minI = 0;
        uint32_t minJ = 1;
        float minD = distPt[1];
        for (uint32_t i = 0; i < N - 1; ++i) {
            float *pDist = distPt.data() + Nstride * ptDistRemap[i];
            for (uint32_t j = i + 1; j < N; ++j) {
                float d = pDist[ptDistRemap[j]];
                if (d < minD) {
                    minD = d;
                    minI = i;
                    minJ = j;
                }
            }
        }
        auto min_i_pt = intersection[minI];
        auto min_j_pt = intersection[minJ];

        // ptDistRemap is not corrupted
        RENSURE(std::abs(
        (min_i_pt.x - min_j_pt.x) * (min_i_pt.x - min_j_pt.x) + (min_i_pt.y - min_j_pt.y) * (min_i_pt.y - min_j_pt.y) -
        minD) < 1.e-6f);
        // drop minJ point
        if (minJ < N - 1) {
            intersection[minJ] = intersection[N - 1];
            ptDistRemap[minJ] = ptDistRemap[N - 1];
        }
        N--;
    }

    // order points
    for (uint32_t i = 0; i < N - 1; ++i) {
        PointT diffI;
        diffI.x = intersection[i + 1].x - intersection[i].x;
        diffI.y = intersection[i + 1].y - intersection[i].y;
        for (uint32_t j = i + 2; j < N; ++j) {
            PointT diffJ;
            diffJ.x = intersection[j].x - intersection[i].x;
            diffJ.y = intersection[j].y - intersection[i].y;
            if ((diffI.x * diffJ.y - diffI.y * diffJ.x) < 0) {
                std::swap(intersection[i + 1], intersection[j]);
                diffI = diffJ;
            }
        }
    }

    intersection.resize(N);
    contours = intersection;
    return ret;
}

}  // namespace rally

#endif  // RALLY_CORE_ALGORITHM_GEOMETRY_IMPL_BOX2D_H
