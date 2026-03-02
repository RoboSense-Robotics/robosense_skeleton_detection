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
#ifndef RALLY_CORE_ALGORITHM_GEOMETRY_BOX2D_H
#define RALLY_CORE_ALGORITHM_GEOMETRY_BOX2D_H

#include "rally/core/algorithm/geometry/aabox2d.h"
#include "rally/core/algorithm/geometry/line_segment2d.h"

namespace rally {

/// @brief Rectangular (undirected) bounding box in 2-D.
/// This class is referential-agnostic, although our convention on the use of
/// the word "heading" in this project (permanently set to be 0 at East)
/// forces us to assume that the X/Y frame here is East/North.
/// For disambiguation, we call the axis of the rectangle parallel to the
/// heading direction the "heading-axis". The size of the heading-axis is
/// called "length", and the size of the axis perpendicular to it "width".
class Box2d {
public:
    using Ptr = std::shared_ptr<Box2d>;

    Box2d() = default;

    /// @brief Constructor which takes the center, heading, length and width.
    /// @param center The center of the rectangular bounding box.
    /// @param heading The angle(in radian) between the x-axis and the heading-axis,
    ///        measured counter-clockwise.
    /// @param length The size of the heading-axis.
    /// @param width The size of the axis perpendicular to the heading-axis.
    Box2d(const ShortArray2f &center, const float heading, const float length, const float width);

    /// @brief Constructor which takes the heading-axis and the width of the box
    /// @param axis The heading-axis
    /// @param width The width of the box, which is taken perpendicularly
    /// to the heading direction.
    Box2d(const LineSegment2d &axis, const float width);

    /// @brief Constructor which takes an AABox2d (axes-aligned box).
    /// @param aabox The input AABox2d.
    explicit Box2d(const AABox2d &aabox);

    /// @brief Creates an axes-aligned Box2d from two opposite corners
    /// @param one_corner One of the corners
    /// @param opposite_corner The opposite corner to the first one
    /// @return An axes-aligned Box2d
    template<typename PointT>
    static Box2d createAABox(const PointT &one_corner, const PointT &opposite_corner);

    /// @brief Getter of the center of the box
    /// @return The center of the box
    const ShortArray2f &center() const { return center_; }

    /// @brief Getter of the x-coordinate of the center of the box
    /// @return The x-coordinate of the center of the box
    float center_x() const { return center_.x; }

    /// @brief Getter of the y-coordinate of the center of the box
    /// @return The y-coordinate of the center of the box
    float center_y() const { return center_.y; }

    /// @brief Getter of the length
    /// @return The length of the heading-axis
    float length() const { return length_; }

    /// @brief Getter of the width
    /// @return The width of the box taken perpendicularly to the heading
    float width() const { return width_; }

    /// @brief Getter of half the length
    /// @return Half the length of the heading-axis
    float half_length() const { return half_length_; }

    /// @brief Getter of half the width
    /// @return Half the width of the box taken perpendicularly to the heading
    float half_width() const { return half_width_; }

    /// @brief Getter of the heading
    /// @return The counter-clockwise angle(in radian) between the x-axis and the heading-axis
    float heading() const { return heading_; }

    /// @brief Getter of the cosine of the heading
    /// @return The cosine of the heading
    float cos_heading() const { return cos_heading_; }

    /// @brief Getter of the sine of the heading
    /// @return The sine of the heading
    float sin_heading() const { return sin_heading_; }

    /// @brief Getter of the area of the box
    /// @return The product of its length and width
    float area() const { return length_ * width_; }

    /// @brief Getter of the size of the diagonal of the box
    /// @return The diagonal size of the box
    float diagonal() const { return std::hypot(length_, width_); }

    /// @brief Getter of the corners of the box
    /// @param corners The vector where the corners are listed
    template<typename PointT>
    void getAllCorners(std::vector<PointT> &corners) const;

    void getAllCorners(std::vector<ShortArray2f> &corners) const { corners = corners_; }

    /// @brief Getter of the corners of the box
    /// @param corners The vector where the corners are listed
    std::vector<ShortArray2f> getAllCorners() const { return corners_; }

    /// @brief Tests points for membership in the box
    /// @param point A point that we wish to test for membership in the box
    /// @return True iff the point is contained in the box
    template<typename PointT>
    bool isPointIn(const PointT &point) const;

    /// @brief Tests points for membership in the boundary of the box
    /// @param point A point that we wish to test for membership in the boundary
    /// @return True iff the point is a boundary point of the box
    template<typename PointT>
    bool isPointOnBoundary(const PointT &point) const;

    /// @brief Determines the distance between the box and a given point
    /// @param point The point whose distance to the box we wish to compute
    /// @return A distance
    template<typename PointT>
    float distanceTo(const PointT &point) const;

    /// @brief Determines the distance between the box and a given line segment
    /// @param line_segment The line segment whose distance to the box we compute
    /// @return A distance
    float distanceTo(const LineSegment2d &line_segment) const;

    /// @brief Determines the distance between two boxes
    /// @param box The box whose distance to this box we want to compute
    /// @return A distance
    float distanceTo(const Box2d &box) const;

    /// @brief Determines whether this box overlaps a given line segment
    /// @param line_segment The line-segment
    /// @return True if they overlap
    bool hasOverlap(const LineSegment2d &line_segment) const;

    /// @brief Determines whether these two boxes overlap
    /// @param line_segment The other box
    /// @return True if they overlap
    bool hasOverlap(const Box2d &box) const;

    float getOverlapArea(const Box2d &box) const;

    /// @brief Gets the smallest axes-aligned box containing the current one
    /// @return An axes-aligned box
    AABox2d getAABox() const;

    /// @brief Rotate from center.
    /// @param rotate_angle Angle(in radian) to rotate.
    void rotateFromCenter(const float rotate_angle);

    /// @brief Shifts this box by a given vector
    /// @param shift_vec The vector determining the shift
    void shift(const ShortArray2f &shift_vec);

    /// @brief Extend the box longitudinally
    /// @param extension_length the length to extend
    void longitudinalExtend(const float extension_length);

    /// @brief Extend the box latitudinally
    /// @param extension_length the length to extend
    void lateralExtend(const float extension_length);

    /// @brief Gets a human-readable description of the box
    /// @return A debug-string
    std::string infos() const;

    float max_x() const { return max_x_; }

    float min_x() const { return min_x_; }

    float max_y() const { return max_y_; }

    float min_y() const { return min_y_; }

private:
    void initCorners();

    ShortArray2f center_;
    float length_{0.f};
    float width_{0.f};
    float half_length_{0.f};
    float half_width_{0.f};
    float heading_{0.f};
    float cos_heading_{0.f};
    float sin_heading_{0.f};

    std::vector<ShortArray2f> corners_;

    float max_x_ = std::numeric_limits<float>::lowest();
    float min_x_ = std::numeric_limits<float>::max();
    float max_y_ = std::numeric_limits<float>::lowest();
    float min_y_ = std::numeric_limits<float>::max();
};

/// @brief types of intersection between rectangles
enum class RectanglesIntersectTypes : uint8_t {
    INTERSECT_NONE = 0, //!< No intersection
    INTERSECT_PARTIAL = 1, //!< There is a partial intersection
    INTERSECT_FULL = 2 //!< One of the rectangle is fully enclosed in the other
};

template<typename PointT>
RectanglesIntersectTypes getInterSection(const Box2d &box1, const Box2d &box2, std::vector<PointT> &contours);

}  // namespace rally

#include "rally/core/algorithm/geometry/impl/box2d.h"

#endif  // RALLY_CORE_ALGORITHM_GEOMETRY_BOX2D_H
