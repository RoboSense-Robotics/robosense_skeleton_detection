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
#ifndef RALLY_CORE_ALGORITHM_GEOMETRY_GEO_UTILS_H
#define RALLY_CORE_ALGORITHM_GEOMETRY_GEO_UTILS_H

#include <cmath>
#include <algorithm>
#include <cstdint>
#include "rally/utils/utils.h"
#include "rally/core/meta/meta.h"

namespace rally {

/// @brief Normalize angle to [-PI, PI).
/// @param[in] rad the original value of the angle(in radian).
/// @return The normalized value of the angle.
float normalizeAngle(const float rad);

/// @brief Wrap angle to [0, 2 * PI).
/// @param[in] rad(in radian) the original value of the angle.
/// @return The wrapped value of the angle.
float wrapAngle(const float rad);

/// @brief a simple template to get rad form a direction
/// @param[in] pt input pt must contains x and y values
/// @return return the radian of the direction
template <typename PointT>
float getRad(const PointT& pt);

/// @brief convert angle in radian to in degree
/// @return return the angle in radian(-Pi, Pi]
float degreeToRad(float degree);

/// @brief convert angle in degree to in radian
/// @return return the angle in degree(-180., 180.]
float radToDegree(float rad);

/// @brief check if a point(with x,y) in a polygon
/// @param[in] pt the tested point
/// @param[in] polygon the tested polygon
/// @return true if the polygon is in the polygon
template<typename PointT1, typename PointT2>
bool inPolygon(const PointT1 &pt, const std::vector<PointT2> &polygon);

/// @brief calculate a contour area
/// @param[in] contour a contour
/// @return the area of the contour
template<typename PointT>
float contourArea(const std::vector<PointT> &contour);

template<typename PointT>
float cross(const PointT &O, const PointT &A, const PointT &B);

template<typename PointT>
void convexHull(std::vector<PointT> P, std::vector<PointT> &polygon);

template<typename PointT>
float getPerpendicularDistance(const PointT &pt, const PointT &lineStart, const PointT &lineEnd);

/// @brief Implements the simplify algorithm.
/// @details The douglas_peucker policy simplifies a linestring, ring or
///          vector of points using the well-known Douglas-Peucker algorithm.
/// @tparam PointT the point type(must contains x and y)
/// @param[in] in_pts pts to be simplified
/// @param[out] out_pts simplified pts
/// @param[in] epsilon used to evaluate
/// @cite For the algorithm, see for example:
///       - http://en.wikipedia.org/wiki/Ramer-Douglas-Peucker_algorithm
template<typename PointT>
void ramerDouglasPeucker(const std::vector<PointT> &in_pts, std::vector<PointT> &out_pts, float epsilon = 0.2);

}  // namespace rally

#include "rally/core/algorithm/geometry/impl/geo_utils.h"

#endif  // RALLY_CORE_ALGORITHM_GEOMETRY_GEO_UTILS_H
