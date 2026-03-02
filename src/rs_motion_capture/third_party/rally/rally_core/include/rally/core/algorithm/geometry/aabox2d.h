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
#ifndef RALLY_CORE_ALGORITHM_GEOMETRY_AABOX2D_H
#define RALLY_CORE_ALGORITHM_GEOMETRY_AABOX2D_H

#include "rally/core/meta/contains_fields.h"
#include "rally/core/algorithm/geometry/geo_utils.h"
#include "rally/core/containers/short_array.h"

namespace rally {

/// @brief Implements a class of (undirected) axes-aligned bounding boxes in 2-D.
class AABox2d {
public:
    using Ptr = std::shared_ptr<AABox2d>;

    /// @brief Default constructor. Creates an axes-aligned box with zero length and width at the origin.
    AABox2d() = default;

    /// @brief Parameterized constructor. Creates an axes-aligned box with given center, length, and width.
    /// @param[in] center The center of the box
    /// @param[in] length The size of the box along the x-axis
    /// @param[in] width The size of the box along the y-axis
    AABox2d(const ShortArray2f &center, const float length = 0.f, const float width = 0.f);

    /// @brief Parameterized constructor. Creates an axes-aligned box from two opposite corners.
    /// @param[in] one_corner One corner of the box
    /// @param[in] opposite_corner The opposite corner to the first one
    AABox2d(const ShortArray2f &one_corner, const ShortArray2f &opposite_corner);

    /// @brief Parameterized constructor. Creates an axes-aligned box containing all points in a given vector.
    /// @param[in] points Vector of points to be included inside the box.
    template<typename PointT>
    explicit AABox2d(const std::vector<PointT> &points);

    /// @brief Getter of center_
    /// @return Center of the box
    const ShortArray2f &center() const noexcept { return center_; }

    /// @brief Getter of x-component of center_
    /// @return x-component of the center of the box
    float center_x() const noexcept { return center_.x; }

    /// @brief Getter of y-component of center_
    /// @return y-component of the center of the box
    float center_y() const noexcept { return center_.y; }

    /// @brief Getter of length_
    /// @return The length of the box
    float length() const noexcept { return length_; }

    /// @brief Getter of width_
    /// @return The width of the box
    float width() const noexcept { return width_; }

    /// @brief Getter of half_length_
    /// @return Half of the length of the box
    float half_length() const noexcept { return half_length_; }

    /// @brief Getter of half_width_
    /// @return Half of the width of the box
    float half_width() const noexcept { return half_width_; }

    /// @brief Getter of area
    /// @return The area of the box
    float area() const noexcept { return length_ * width_; }

    /// @brief Returns the minimum x-coordinate of the box
    /// @return x-coordinate
    float min_x() const noexcept { return center_.x - half_length_; }

    /// @brief Returns the maximum x-coordinate of the box
    /// @return x-coordinate
    float max_x() const noexcept { return center_.x + half_length_; }

    /// @brief Returns the minimum y-coordinate of the box
    /// @return y-coordinate
    float min_y() const noexcept { return center_.y - half_width_; }

    /// @brief Returns the maximum y-coordinate of the box
    /// @return y-coordinate
    float max_y() const { return center_.y + half_width_; }

    /// @brief Gets all corners in counter clockwise order.
    /// @param corners Output where the corners are written
    template<typename PointT>
    void getAllCorners(std::vector<PointT> &corners) const;

    /// @brief Determines whether a given point is in the box.
    /// @param point The point we wish to test for containment in the box
    template<typename PointT>
    bool isPointIn(const PointT &point) const;

    /// @brief Determines whether a given point is on the boundary of the box.
    /// @param point The point we wish to test for boundary membership
    template<typename PointT>
    bool isPointOnBoundary(const PointT &point) const;

    /// @brief Determines the distance between a point and the box.
    /// @param point The point whose distance to the box we wish to determine.
    template<typename PointT>
    float distanceTo(const PointT &point) const;

    /// @brief Determines the distance between two boxes.
    /// @param box Another box.
    float distanceTo(const AABox2d &box) const;

    /// @brief Determines whether two boxes overlap.
    /// @param box Another box
    bool hasOverlap(const AABox2d &box) const;

    /// @brief Shift the center of AABox by the input vector.
    /// @param shift_vec The vector by which we wish to shift the box
    void shift(const ShortArray2f &shift_vec);

    /// @brief Changes box to include another given box, as well as the current one.
    void mergeFrom(const AABox2d &other_box);

    /// @brief Changes box to include a given point, as well as the current box.
    /// @param[in] other_point Another point
    template<typename PointT>
    void mergeFrom(const PointT &other_point);

    /// @brief Gets a human-readable debug string
    /// @return A string
    std::string infos() const;

private:
    ShortArray2f center_;
    float length_{0.f};
    float width_{0.f};
    float half_length_{0.f};
    float half_width_{0.f};
};

}  // namespace rally

#include "rally/core/algorithm/geometry/impl/aabox2d.h"

#endif  // RALLY_CORE_ALGORITHM_GEOMETRY_AABOX2D_H
