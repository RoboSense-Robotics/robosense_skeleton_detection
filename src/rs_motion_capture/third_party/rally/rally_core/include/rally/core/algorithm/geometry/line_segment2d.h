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
#ifndef RALLY_CORE_ALGORITHM_GEOMETRY_LINE_SEGMENT2D_H
#define RALLY_CORE_ALGORITHM_GEOMETRY_LINE_SEGMENT2D_H

#include "rally/core/meta/contains_fields.h"
#include "rally/core/algorithm/geometry/geo_utils.h"
#include "rally/core/containers/short_array.h"

namespace rally {

/// @brief Line segment in 2-D.
class LineSegment2d {
public:
    using Ptr = std::shared_ptr<LineSegment2d>;

    /// @brief Empty constructor.
    LineSegment2d();

    /// @brief Constructor with start point and end point.
    /// @param start The start point of the line segment.
    /// @param end The end point of the line segment.
    template<typename PointT>
    LineSegment2d(const PointT &start, const PointT &end);

    LineSegment2d(const ShortArray2f &start, const ShortArray2f &end);

    /// @brief Get the start point.
    /// @return The start point of the line segment.
    const ShortArray2f &start() const { return start_; }

    /// @brief Get the end point.
    /// @return The end point of the line segment.
    const ShortArray2f &end() const { return end_; }

    /// @brief Get the unit direction from the start point to the end point.
    /// @return The start point of the line segment.
    const ShortArray2f &unit_direction() const { return unit_direction_; }

    /// @brief Get the center of the line segment.
    /// @return The center of the line segment.
    ShortArray2f center() const { return (start_ + end_) / 2.f; }

    /// @brief Get a new line-segment with the same start point, but rotated
    /// counterclock-wise by the given amount.
    /// @return The rotated line-segment's end-point.
    ShortArray2f rotate(const float rad);

    /// @brief Get the heading of the line segment.
    /// @return The heading, which is the angle between unit direction and x-axis.
    float heading() const { return heading_; }

    /// @brief Get the cosine of the heading.
    /// @return The cosine of the heading.
    float cos_heading() const { return unit_direction_.x; }

    /// @brief Get the sine of the heading.
    /// @return The sine of the heading.
    float sin_heading() const { return unit_direction_.y; }

    /// @brief Get the length of the line segment.
    /// @return The length of the line segment.
    float length() const { return length_; }

    /// @brief Get the square of length of the line segment.
    /// @return The square of length of the line segment.
    float length_sqr() const { return length_ * length_; }

    /// @brief Compute the shortest distance from a point on the line segment to a point in 2-D.
    /// @param point The point to compute the distance to.
    /// @return The shortest distance from points on the line segment to point.
    template<typename PointT>
    float distanceTo(const PointT &point) const;

    /// @brief Compute the shortest distance from a point on the line segment
    ///        to a point in 2-D, and get the nearest point on the line segment.
    /// @param[in] point The point to compute the distance to.
    /// @param[out] nearest_pt The nearest point on the line segment
    ///             to the input point.
    /// @return The shortest distance from points on the line segment
    ///         to the input point.
    template<typename PointT>
    float distanceTo(const PointT &point, PointT &nearest_pt) const;

    /// @brief Compute the square of the shortest distance from a point
    ///        on the line segment to a point in 2-D.
    /// @param[in] point The point to compute the squared of the distance to.
    /// @return The square of the shortest distance from points
    ///         on the line segment to the input point.
    template<typename PointT>
    float distanceSquareTo(const PointT &point) const;

    /// @brief Compute the square of the shortest distance from a point
    ///        on the line segment to a point in 2-D,
    ///        and get the nearest point on the line segment.
    /// @param[in] point The point to compute the squared of the distance to.
    /// @param[out] nearest_pt The nearest point on the line segment
    ///             to the input point.
    /// @return The shortest distance from points on the line segment
    ///         to the input point.
    template<typename PointT>
    float distanceSquareTo(const PointT &point, PointT &nearest_pt) const;

    /// @brief Check if a point is within the line segment.
    /// @param point The point to check if it is within the line segment.
    /// @return Whether the input point is within the line segment or not.
    template<typename PointT>
    bool isPointIn(const PointT &point) const;

    bool isPointIn(const ShortArray2f &point) const;

    /// @brief Check if the line segment has an intersect
    ///        with another line segment in 2-D.
    /// @param other_segment The line segment to check if it has an intersect.
    /// @return Whether the line segment has an intersect
    ///         with the input other_segment.
    bool hasIntersect(const LineSegment2d &other_segment) const;

    /// @brief Compute the intersect with another line segment in 2-D if any.
    /// @param[in] other_segment The line segment to compute the intersect.
    /// @param[out] point the computed intersect between the line segment and
    ///             the input other_segment.
    /// @return Whether the line segment has an intersect with the input other_segment.
    template<typename PointT>
    bool getIntersect(const LineSegment2d &other_segment, PointT &point) const;

    /// @brief Compute the projection of a vector onto the line segment.
    /// @param point The end of the vector (starting from the start point of the
    ///        line segment) to compute the projection onto the line segment.
    /// @return The projection of the vector, which is from the start point of
    ///        the line segment to the input point, onto the unit direction.
    template<typename PointT>
    float projectOntoUnit(const PointT &point) const;

    float projectOntoUnit(const ShortArray2f &point) const { return innerProd(unit_direction_, point - start_); }

    /// @brief Compute the cross product of a vector onto the line segment.
    /// @param point The end of the vector (starting from the start point of the
    ///        line segment) to compute the cross product onto the line segment.
    /// @return The cross product of the unit direction and
    ///         the vector, which is from the start point of
    ///         the line segment to the input point.
    template<typename PointT>
    float productOntoUnit(const PointT &point) const;

    float productOntoUnit(const ShortArray2f &point) const { return crossProd(unit_direction_, point - start_); }

    /// @brief Compute perpendicular foot of a point in 2-D on the straight line
    ///        expanded from the line segment.
    /// @param[in] point The point to compute the perpendicular foot from.
    /// @param[out] foot_point The computed perpendicular foot from the input point to
    ///             the straight line expanded from the line segment.
    /// @return The distance from the input point to the perpendicular foot.
    template<typename PointT>
    float getPerpendicularFoot(const PointT &point, PointT &foot_point) const;

    /// @brief Get the debug string including the essential information.
    /// @return Information of the line segment for debugging.
    std::string infos() const;

private:
    ShortArray2f start_;
    ShortArray2f end_;
    ShortArray2f unit_direction_;
    float heading_ = 0.f;
    float length_ = 0.f;
};

}  // namespace rally

#include "rally/core/algorithm/geometry/impl/line_segment2d.h"

#endif  // RALLY_CORE_ALGORITHM_GEOMETRY_LINE_SEGMENT2D_H
