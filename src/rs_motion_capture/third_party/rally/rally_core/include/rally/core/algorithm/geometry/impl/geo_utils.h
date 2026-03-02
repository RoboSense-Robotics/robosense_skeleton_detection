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
#ifndef RALLY_CORE_ALGORITHM_GEOMETRY_IMPL_GEO_UTILS_H
#define RALLY_CORE_ALGORITHM_GEOMETRY_IMPL_GEO_UTILS_H

namespace rally {

template <typename PointT>
inline float getRad(const PointT& pt) {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>::value && rally::details::has_y<PointT>::value);
    return std::atan2(pt.y, pt.x);
}

template<typename PointT1, typename PointT2>
inline bool inPolygon(const PointT1 &pt, const std::vector<PointT2> &polygon) {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT1>().value && rally::details::has_y<PointT1>().value &&
                        rally::details::has_x<PointT2>().value && rally::details::has_y<PointT2>().value);

    bool c = false;
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        if (((polygon[i].y > pt.y) != (polygon[j].y > pt.y)) &&
            (pt.x < (polygon[j].x - polygon[i].x) * (pt.y - polygon[i].y) /
                    (polygon[j].y - polygon[i].y) + polygon[i].x))
            c = !c;
    }
    return c;
}

template<typename PointT>
inline float contourArea(const std::vector<PointT> &contour) {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>().value && rally::details::has_y<PointT>().value);
    size_t npoints = contour.size();

    if (npoints == 0) {
        return 0.f;
    }

    float a00 = 0.f;
    auto prev = contour[npoints - 1];

    for (size_t i = 0; i < npoints; i++) {
        const auto &p = contour[i];
        a00 += prev.x * p.y - prev.y * p.x;
        prev = p;
    }

    a00 *= 0.5f;
    a00 = std::abs(a00);

    return a00;
}

template<typename PointT>
inline float cross(const PointT &O, const PointT &A, const PointT &B) {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>().value && rally::details::has_y<PointT>().value);
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

template<typename PointT>
inline void convexHull(std::vector<PointT> P, std::vector<PointT> &polygon) {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>().value && rally::details::has_y<PointT>().value);
    int32_t n = static_cast<int32_t>(P.size());
    int32_t m = 0;
    if (n < 3) {
        polygon = P;
        return;
    }

    polygon.resize(2 * n);
    // sorted as cxvalue
    std::sort(P.begin(), P.end(), [&](const PointT &lhs, const PointT &rhs) {
        return (lhs.x < rhs.x || (lhs.x == rhs.x && lhs.y < rhs.y));
    });

    for (int32_t i = 0; i < n; ++i) {
        while (m >= 2 && cross(polygon[m - 2], polygon[m - 1], P[i]) <= 0) {
            m--;
        }
        polygon[m++] = P[i];
    }
    for (int32_t i = n - 2, t = m + 1; i >= 0; --i) {
        while (m >= t && cross(polygon[m - 2], polygon[m - 1], P[i]) <= 0) {
            m--;
        }
        polygon[m++] = P[i];
    }
    m--;

    polygon.resize(m);
}

template<typename PointT>
inline float getPerpendicularDistance(const PointT &pt, const PointT &lineStart, const PointT &lineEnd) {
    RALLY_STATIC_ASSERT(rally::details::has_x<PointT>().value && rally::details::has_y<PointT>().value);
    float dx = lineEnd.x - lineStart.x;
    float dy = lineEnd.y - lineStart.y;

    // Normalise
    float mag = std::sqrt(dx * dx + dy * dy);
    if (mag > 0.0) {
        dx /= mag;
        dy /= mag;
    }

    float pvx = pt.x - lineStart.x;
    float pvy = pt.y - lineStart.y;

    // Get dot product (project pv onto normalized direction)
    float pvdot = dx * pvx + dy * pvy;

    // Scale line direction vector
    float dsx = pvdot * dx;
    float dsy = pvdot * dy;

    // Subtract this from pv
    float ax = pvx - dsx;
    float ay = pvy - dsy;

    return std::sqrt(ax * ax + ay * ay);
}

template<typename PointT>
inline void ramerDouglasPeucker(const std::vector<PointT> &in_pts, std::vector<PointT> &out_pts, float epsilon) {
    if (in_pts.size() <= 3) {
        out_pts = in_pts;
        return;
    }
    // Find the point with the maximum distance from line between start and end
    float dmax = 0.f;
    size_t index = 0;
    size_t end = in_pts.size() - 1;
    for (size_t i = 1; i < end; i++) {
        float d = getPerpendicularDistance(in_pts[i], in_pts[0], in_pts[end]);
        if (d > dmax) {
            index = i;
            dmax = d;
        }
    }
    // If max distance is greater than epsilon, recursively simplify
    if (dmax > epsilon) {
        // Recursive call
        std::vector<PointT> recResults1;
        std::vector<PointT> recResults2;
        std::vector<PointT> firstLine(in_pts.begin(), in_pts.begin() + index + 1);
        std::vector<PointT> lastLine(in_pts.begin() + index, in_pts.end());
        ramerDouglasPeucker(firstLine, recResults1, epsilon);
        ramerDouglasPeucker(lastLine, recResults2, epsilon);

        // Build the result list
        out_pts.assign(recResults1.begin(), recResults1.end() - 1);
        out_pts.insert(out_pts.end(), recResults2.begin(), recResults2.end());
        RENSURE(out_pts.size() >= 3);
    } else {
        //Just return start and end points
        out_pts.clear();
        out_pts.push_back(in_pts[0]);
        out_pts.push_back(in_pts[end]);
    }
}

}  // namespace rally

#endif  // RALLY_CORE_ALGORITHM_GEOMETRY_IMPL_GEO_UTILS_H
