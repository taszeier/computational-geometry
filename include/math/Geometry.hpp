#pragma once

#include <optional>

#include "common/Common.hpp"
#include "math/HalfPlaneRelation.hpp"
#include "math/Side.hpp"
#include "math/primitives/Ball.hpp"
#include "math/primitives/Box.hpp"
#include "math/primitives/HalfPlane.hpp"
#include "math/primitives/Hyperplane.hpp"
#include "math/primitives/Line.hpp"
#include "math/primitives/LineSegment.hpp"
#include "primitives/HalfLine.hpp"

namespace compg {

    PointSide FindSide(const Vertex2D& a, const Vertex2D& b, const Vertex2D& v, Float epsilon = EPSILON);
    PointSide FindSide(const Line2D& line, const Vertex2D& v, Float epsilon = EPSILON);
    PointSide FindSide(const LineSegment2D& segment, const Vertex2D& v, Float epsilon = EPSILON);
    PointSide FindSide(const HalfLine2D& halfLine, const Vertex2D& v, Float epsilon = EPSILON);

    std::optional<Vertex2D> FindIntersection(const Line2D& l1, const Line2D& l2, Float epsilon = EPSILON);
    std::optional<Vertex2D> FindIntersection(const LineSegment2D& l1, const LineSegment2D& l2, Float epsilon = EPSILON);
    std::optional<Vertex2D> FindIntersection(const LineSegment2D& segment, const Line2D& line, Float epsilon = EPSILON);
    std::optional<Vertex2D> FindIntersection(const Line2D& line, const LineSegment2D& segment, Float epsilon = EPSILON);
    std::optional<Vertex2D>
    FindIntersection(const Hyperplane<2UZ>& h1, const Hyperplane<2UZ>& h2, Float epsilon = EPSILON);
    std::optional<Vertex2D>
    FindIntersection(const AxesAlignedHyperplane<2UZ>& hl, const LineSegment2D& segment, Float epsilon = EPSILON);
    std::optional<Vertex2D>
    FindIntersection(const Hyperplane2D& hl, const LineSegment2D& segment, Float epsilon = EPSILON);

    std::optional<Vertex2D> FindIntersection(const HalfLine2D& hl, const Line2D& l, Float epsilon = EPSILON);
    std::optional<Vertex2D> FindIntersection(const Line2D& l, const HalfLine2D& hl, Float epsilon = EPSILON);
    std::optional<Vertex2D> FindIntersection(const HalfLine2D& hl, const LineSegment2D& s, Float epsilon = EPSILON);
    std::optional<Vertex2D> FindIntersection(const LineSegment2D& s, const HalfLine2D& hl, Float epsilon = EPSILON);
    std::optional<Vertex2D> FindIntersection(const HalfLine2D& hl1, const HalfLine2D& hl2, Float epsilon = EPSILON);

    std::optional<Vertex3D>
    FindIntersection(const Hyperplane3D& hyperplane, const Vertex3D& a, const Vertex3D& b, Float epsilon = EPSILON);

    inline std::optional<Vertex3D>
    FindIntersection(const Hyperplane3D& hyperplane, const LineSegment3D& segment, Float epsilon = EPSILON) {
        return FindIntersection(hyperplane, segment[0], segment[1], epsilon);
    }

    Float Angle180(const Vertex2D& v1, const Vertex2D& v2);
    Float Angle360(const Vertex2D& v1, const Vertex2D& v2);

    constexpr Float RadiansToDegrees(Float radians) {
        return 180.0 / math::PI * radians;
    }

    constexpr Float DegreesToRadians(Float degrees) {
        return math::PI / 180.0 * degrees;
    }

    template <std::size_t K>
    Float FindSignedDistance(const Hyperplane<K>& plane, const Vertex<K>& v) {
        const auto& p = plane.GetOrigin();
        const auto& n = plane.GetNormal();
        return (v - p).dot(n);
    }

    template <std::size_t K>
    Float FindSignedDistance(const HalfPlane<Hyperplane<K>>& halfPlane, const Vertex<K>& v) {
        return FindSignedDistance(halfPlane.Plane, v);
    }

    template <std::size_t K>
    Float FindSignedDistance(const HalfPlane<AxesAlignedHyperplane<K>>& halfPlane, const Vertex<K>& v) {
        const auto i = halfPlane.Plane.GetAxisIndex();
        auto d = v[i] - halfPlane.Plane.GetIntersection();
        if (halfPlane.NormalSide == Side::Negative) {
            d *= -1;
        }
        return d;
    }

    template <std::size_t K>
    PointSide FindSide(const HalfPlane<Hyperplane<K>>& halfPlane, const Vertex<K>& v, Float epsilon = EPSILON) {
        const Float d = FindSignedDistance<K>(halfPlane.Plane, v);
        if (math::IsZero(d, epsilon)) {
            return PointSide::Collinear;
        }
        return d > epsilon ? PointSide::Positive : PointSide::Negative;
    }

    template <std::size_t K>
    PointSide
    FindSide(const HalfPlane<AxesAlignedHyperplane<K>>& halfPlane, const Vertex<K>& v, Float epsilon = EPSILON) {
        const Float distance = FindSignedDistance<K>(halfPlane, v);
        if (math::IsZero(distance, epsilon)) {
            return PointSide::Collinear;
        }
        return distance > epsilon ? PointSide::Positive : PointSide::Negative;
    }

    template <std::size_t K>
    HalfPlaneRelation
    FindHalfPlaneRelation(const HalfPlane<Hyperplane<K>>& halfPlane, const Box<K>& box, Float epsilon = EPSILON) {
        auto lowerSide = FindSide(halfPlane, box.GetLowerCorner(), epsilon);
        auto upperSide = FindSide(halfPlane, box.GetUpperCorner(), epsilon);
        if (lowerSide == PointSide::Negative && upperSide == PointSide::Negative) {
            return HalfPlaneRelation::Negative;
        }
        if (lowerSide == PointSide::Positive && upperSide == PointSide::Positive) {
            return HalfPlaneRelation::Positive;
        }
        return HalfPlaneRelation::Intersection;
    }

    template <std::size_t K>
    Vertex<K> FindClosestPoint(const Hyperplane<K>& plane, const Vertex<K>& v) {
        const auto& p = plane.GetOrigin();
        const auto& n = plane.GetNormal();
        return v - (v - p).dot(n) * n;
    }

    template <typename HyperplaneType>
    HalfPlaneRelation FindHalfPlaneRelation(
        const HalfPlane<HyperplaneType>& halfPlane, const Ball<HyperplaneType::K>& ball, Float epsilon = EPSILON
    ) {
        const Float distance = FindSignedDistance<HyperplaneType::K>(halfPlane, ball.GetCenter());
        if (math::IsZero(distance, ball.GetRadius() + epsilon)) {
            return HalfPlaneRelation::Intersection;
        }
        return distance > ball.GetRadius() + epsilon ? HalfPlaneRelation::Positive : HalfPlaneRelation::Negative;
    }

    inline Vertex2D ProjectOnto(const Vertex2D& vertex, const Line2D& line) {
        const auto a = line[0];
        const auto b = line[1];
        const auto ab = b - a;
        const auto av = vertex - a;
        const auto t = av.dot(ab) / ab.squaredNorm();

        return a + t * ab;
    }

    inline bool IsHorizontal(const Hyperplane<2UZ>& hyperplane, Float epsilon = EPSILON) {
        return math::IsZero(hyperplane.GetNormal()[0], epsilon);
    }

    inline bool IsVertical(const Hyperplane<2UZ>& hyperplane, Float epsilon = EPSILON) {
        return math::IsZero(hyperplane.GetNormal()[1], epsilon);
    }

    inline Line2D CreateBisector(const Vertex2D& a, const Vertex2D& b) {
        const auto ab = b - a;
        const Vertex2D normal{-ab[1], ab[0]};
        const auto center = (a + b) * 0.5;
        return Line2D{center, center + normal};
    }

    inline Line2D CreateBisector(const LineSegment2D& segment) {
        return CreateBisector(segment[0], segment[1]);
    }

    Vertex2D Rotate(const Vertex2D& v, Float angle);
    Hyperplane<2UZ> Rotate(const Hyperplane<2UZ>& hyperplane, Float angle);

    template <std::size_t K>
    bool AreEqual(const Hyperplane<K>& a, const Hyperplane<K>& b, Float epsilon = EPSILON) {
        return AreEqual(a.GetNormal(), b.GetNormal(), epsilon)
               && FindSide<K>(HalfPlane<Hyperplane<K>>{a}, b.GetOrigin(), epsilon) == PointSide::Collinear;
    }

    inline std::optional<Vertex2D> FindCircumcenter(const Vertex2D& v0, const Vertex2D& v1, const Vertex2D& v2) {
        const auto bisector12 = CreateBisector(v0, v1);
        const auto bisector13 = CreateBisector(v0, v2);
        return FindIntersection(bisector12, bisector13);
    }

    inline std::optional<Ball<2UZ>> FindCircumcircle(const Vertex2D& v0, const Vertex2D& v1, const Vertex2D& v2) {
        return FindCircumcenter(v0, v1, v2).transform([&v0](const Vertex2D& center) {
            return Ball<2UZ>{center, (v0 - center).norm()};
        });
    }

    /**
     * @brief Find the area of a triangle.
     * @details The corners can be passed in any order.
     * @param v0 A corner of the triangle.
     * @param v1 A corner of the triangle.
     * @param v2 A corner of the triangle.
     * @return The area of the triangle defined by the three vertices.
     */
    Float FindTriangleArea(const Vertex2D& v0, const Vertex2D& v1, const Vertex2D& v2);

    /**
     * @brief Find the area of a convex polygon given its corners.
     * @details The corners can be in an arbitrary order.
     * @param vertices The corners of the convex polygon.
     * @return The area of the convex polygon.
     */
    Float FindConvexPolygonAreaUnordered(const std::vector<Vertex2D>& vertices);

    /**
     * @brief Find the area of a convex polygon given its corners.
     * @details The corners must be in either clockwise or anti-clockwise order.
     * @param vertices The sorted corners of the convex polygon.
     * @return The area of the convex polygon.
     */
    Float FindConvexPolygonArea(const std::vector<Vertex2D>& vertices);

    bool Contains(const Line3D& line, const Vertex3D& v, Float epsilon = EPSILON);
    bool Contains(const Vertex3D& a, const Vertex3D& b, const Vertex3D& v, Float epsilon = EPSILON);
} // namespace compg
