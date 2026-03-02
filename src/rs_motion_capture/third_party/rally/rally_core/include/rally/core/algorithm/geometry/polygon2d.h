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
#ifndef RALLY_CORE_ALGORITHM_GEOMETRY_POLYGON2D_H
#define RALLY_CORE_ALGORITHM_GEOMETRY_POLYGON2D_H

#include "rally/core/algorithm/geometry/box2d.h"
#include "rally/core/algorithm/geometry/line_segment2d.h"

namespace rally {

/// @brief The class of polygon in 2-D.
class Polygon2d {
public:
    using Ptr = std::shared_ptr<Polygon2d>;

    /// @brief Empty constructor.
    Polygon2d() = default;

    /// @brief Constructor which takes a box.
    /// @param box The box to construct the polygon.
    explicit Polygon2d(const Box2d &box);

    /// @brief Constructor which takes a vector of points as its vertices.
    /// @param points The points to construct the polygon.
    template<typename PointT>
    explicit Polygon2d(std::vector<PointT> points);

    Polygon2d(std::vector<ShortArray2f> points);

    /// @brief Get the vertices of the polygon.
    /// @return The vertices of the polygon.
    const std::vector<ShortArray2f> &points() const { return points_; }

    /// @brief Get the edges of the polygon.
    /// @return The edges of the polygon.
    const std::vector<LineSegment2d> &line_segments() const { return line_segments_; }

    /// @brief Get the number of vertices of the polygon.
    /// @return The number of vertices of the polygon.
    uint32_t num_points() const { return num_points_; }

    /// @brief Check if the polygon is convex.
    /// @return Whether the polygon is convex or not.
    bool is_convex() const { return is_convex_; }

    /// @brief Get the area of the polygon.
    /// @return The area of the polygon.
    float area() const { return area_; }

    /// @brief Compute the distance from a point to the boundary of the polygon.
    ///        This distance is equal to the minimal distance from the point
    ///        to the edges of the polygon.
    /// @param point The point to compute whose distance to the polygon.
    /// @return The distance from the point to the polygon's boundary.
    template<typename PointT>
    float distanceToBoundary(const PointT &point) const;

    /// @brief Compute the distance from a point to the polygon. If the point is
    ///        within the polygon, return 0. Otherwise, this distance is
    ///        the minimal distance from the point to the edges of the polygon.
    /// @param point The point to compute whose distance to the polygon.
    /// @return The distance from the point to the polygon.
    template<typename PointT>
    float distanceTo(const PointT &point) const;

    /// @brief Compute the distance from a line segment to the polygon.
    ///        If the line segment is within the polygon, or it has intersect with
    ///        the polygon, return 0. Otherwise, this distance is
    ///        the minimal distance between the distances from the two ends
    ///        of the line segment to the polygon.
    /// @param line_segment The line segment to compute whose distance to the polygon.
    /// @return The distance from the line segment to the polygon.
    float distanceTo(const LineSegment2d &line_segment) const;

    /// @brief Compute the distance from a box to the polygon.
    ///        If the box is within the polygon, or it has overlap with
    ///        the polygon, return 0. Otherwise, this distance is
    ///        the minimal distance among the distances from the edges
    ///        of the box to the polygon.
    /// @param box The box to compute whose distance to the polygon.
    /// @return The distance from the box to the polygon.
    float distanceTo(const Box2d &box) const;

    /// @brief Compute the distance from another polygon to the polygon.
    ///        If the other polygon is within this polygon, or it has overlap with
    ///        this polygon, return 0. Otherwise, this distance is
    ///        the minimal distance among the distances from the edges
    ///        of the other polygon to this polygon.
    /// @param polygon The polygon to compute whose distance to this polygon.
    /// @return The distance from the other polygon to this polygon.
    float distanceTo(const Polygon2d &polygon) const;

    /// @brief Compute the square of distance from a point to the polygon.
    ///        If the point is within the polygon, return 0. Otherwise,
    ///        this square of distance is the minimal square of distance from
    ///        the point to the edges of the polygon.
    /// @param point The point to compute whose square of distance to the polygon.
    /// @return The square of distance from the point to the polygon.
    template<typename PointT>
    float distanceSquareTo(const PointT &point) const;

    /// @brief Check if a point is within the polygon.
    /// @param point The target point. To check if it is within the polygon.
    /// @return Whether a point is within the polygon or not.
    template<typename PointT>
    bool isPointIn(const PointT &point) const;

    /// @brief Check if a point is on the boundary of the polygon.
    /// @param point The target point. To check if it is on the boundary of the polygon.
    /// @return Whether a point is on the boundary of the polygon or not.
    template<typename PointT>
    bool isPointOnBoundary(const PointT &point) const;

    /// @brief Check if the polygon contains a line segment.
    /// @param line_segment The target line segment. To check if the polygon contains it.
    /// @return Whether the polygon contains the line segment or not.
    bool contains(const LineSegment2d &line_segment) const;

    /// @brief Check if the polygon contains another polygon.
    /// @param polygon The target polygon. To check if this polygon contains it.
    /// @return Whether this polygon contains another polygon or not.
    bool contains(const Polygon2d &polygon) const;

    /// @brief Compute the convex hull of a group of points.
    /// @param[in] points The target points. To compute the convex hull of them.
    /// @param[out] polygon The convex hull of the points.
    /// @return If successfully compute the convex hull.
    template<typename PointT>
    static bool computeConvexHull(const std::vector<PointT> &points, Polygon2d &polygon);

    /// @brief Check if a line segment has overlap with this polygon.
    /// @param line_segment The target line segment. To check if it has
    ///        overlap with this polygon.
    /// @return Whether the target line segment has overlap with this
    ///         polygon or not.
    bool hasOverlap(const LineSegment2d &line_segment) const;

    /// @brief Get the overlap of a line segment and this polygon. If they have
    ///        overlap, output the two ends of the overlapped line segment.
    /// @param[in] line_segment The target line segment. To get its overlap with this polygon.
    /// @param[out] first First end of the overlapped line segment.
    /// @param[out] second Second end of the overlapped line segment.
    /// @return If the target line segment has overlap with this polygon.
    bool getOverlap(const LineSegment2d &line_segment, ShortArray2f &first, ShortArray2f &last) const;

    /// @brief Get all vertices of the polygon
    /// @param[out] All vertices of the polygon
    template<typename PointT>
    void getAllVertices(std::vector<PointT> &vertices) const;

    void getAllVertices(std::vector<ShortArray2f> &vertices) const { vertices = points_; }

    /// @brief Get all overlapped line segments of a line segment and this polygon.
    ///        There are possibly multiple overlapped line segments if this
    ///        polygon is not convex.
    /// @param line_segment The target line segment. To get its all overlapped
    ///        line segments with this polygon.
    /// @return A group of overlapped line segments.
    std::vector<LineSegment2d> getAllOverlaps(const LineSegment2d &line_segment) const;

    /// @brief Check if this polygon has overlap with another polygon.
    /// @param polygon The target polygon. To check if it has overlap with this polygon.
    /// @return If this polygon has overlap with another polygon.
    bool hasOverlap(const Polygon2d &polygon) const;

    /// @brief Compute the overlap of this polygon and the other polygon if any.
    ///        Note: this function only works for computing overlap between
    ///        two convex polygons.
    /// @param other_polygon The target polygon. To compute its overlap with this polygon.
    /// @param overlap_polygon The overlapped polygon.
    /// @param If there is an overlapped polygon.
    bool computeOverlap(const Polygon2d &other_polygon, Polygon2d &overlap_polygon) const;

    /// @brief Compute intersection over union ratio of this polygon and the other
    ///        polygon. Note: this function only works for computing overlap
    ///        between two convex polygons.
    /// @param other_polygon The target polygon. To compute its overlap with this polygon.
    /// @return A value between 0.0 and 1.0, meaning no intersection to fully overlaping
    float computeIoU(const Polygon2d &other_polygon) const;

    /// @brief Get the axis-aligned bound box of the polygon.
    /// @return The axis-aligned bound box of the polygon.
    AABox2d getAABoundingBox() const;

    /// @brief Get the bound box according to a heading.
    /// @param heading The specified heading of the bounding box.
    /// @return The bound box according to the specified heading.
    Box2d getBoundingBoxWithHeading(const float heading) const;

    /// @brief Get the bounding box with the minimal area.
    /// @return The bounding box with the minimal area.
    Box2d getMinAreaBoundingBox() const;

    /// @brief Get the extreme points along a heading direction.
    /// @param heading The specified heading.
    /// @param first The point on the boundary of this polygon with the minimal
    ///        projection onto the heading direction.
    /// @param last The point on the boundary of this polygon with the maximal
    ///        projection onto the heading direction.
    void extremePoints(const float heading, ShortArray2f &first, ShortArray2f &last) const;

    /// @brief Expand this polygon by a distance.
    /// @param distance The specified distance. To expand this polygon by it.
    /// @return The polygon after expansion.
    Polygon2d expandByDistance(const float distance) const;

    /// @brief Get a string containing essential information about the polygon
    ///        for debugging purpose.
    /// @return Essential information about the polygon for debugging purpose.
    std::string infos() const;

    float min_x() const { return min_x_; }

    float max_x() const { return max_x_; }

    float min_y() const { return min_y_; }

    float max_y() const { return max_y_; }

protected:
    void buildFromPoints();

    uint32_t next(uint32_t at) const { return at >= num_points_ - 1 ? 0 : at + 1; }

    uint32_t prev(uint32_t at) const { return at == 0 ? num_points_ - 1 : at - 1; }

    static bool clipConvexHull(const LineSegment2d &line_segment, std::vector<ShortArray2f> &points);

    std::vector<ShortArray2f> points_;
    uint32_t num_points_{0};
    std::vector<LineSegment2d> line_segments_;
    bool is_convex_{false};
    float area_{0.f};
    float min_x_{0.f};
    float max_x_{0.f};
    float min_y_{0.f};
    float max_y_{0.f};
};

}  // namespace rally

#include "rally/core/algorithm/geometry/impl/polygon.h"

#endif  // RALLY_CORE_ALGORITHM_GEOMETRY_POLYGON2D_H
