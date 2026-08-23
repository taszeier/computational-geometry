#pragma once
#include "common/Common.hpp"
#include "math/Geometry.hpp"
#include "math/primitives/Line.hpp"
#include "math/primitives/LineSegment.hpp"

namespace compg {
    class VertexBelowComparator {
    public:
        static bool operator()(const Vertex2D& v1, const Vertex2D& v2) {
            return v1[1] < v2[1] || (v1[1] == v2[1] && v1[0] > v2[0]);
        }

        static bool Compare(const Vertex2D& v1, const Vertex2D& v2) {
            return operator()(v1, v2);
        }
    };

    class VertexAboveComparator {
    public:
        static bool operator()(const Vertex2D& v1, const Vertex2D& v2) {
            return VertexBelowComparator::operator()(v2, v1);
        }

        static bool Compare(const Vertex2D& v1, const Vertex2D& v2) {
            return operator()(v1, v2);
        }
    };
    inline const auto& UpperVertex(const LineSegment2D& l) {
        return VertexBelowComparator::Compare(l[1], l[0]) ? l[0] : l[1];
    }

    inline const auto& LowerVertex(const LineSegment2D& l) {
        return VertexBelowComparator::Compare(l[1], l[0]) ? l[1] : l[0];
    }

    /**
     * @brief Compare line segments using the x-coordinate of the intersection
     * points on the sweep line. If the line segments intersect on the sweep
     * line, the angles between the line segments and the sweep line are used to
     * determine the order.
     */
    class LineSegmentLeftToRightComparator {
    public:
        explicit LineSegmentLeftToRightComparator(const Vertex2D& eventPoint, Float epsilon = EPSILON)
            : EventPoint(eventPoint)
            , Epsilon(epsilon) {}

    public:
        bool operator()(const LineSegment2D& l1, const LineSegment2D& l2) const {
            const Line2D sweepLine = MakeLine2DFromY(EventPoint.y());

            const std::optional<Vertex2D> maybeIntersection1 = FindIntersection(l1, sweepLine);
            const std::optional<Vertex2D> maybeIntersection2 = FindIntersection(l2, sweepLine);

            const Vertex2D& intersection1 = maybeIntersection1.value_or(EventPoint);
            const Vertex2D& intersection2 = maybeIntersection2.value_or(EventPoint);

            return AreEqual(intersection1, intersection2, Epsilon) ? IsAngleSmaller(l1, l2, intersection1)
                                                                   : intersection1[0] < intersection2[0];
        }

    private:
        bool IsAngleSmaller(const LineSegment2D& l1, const LineSegment2D& l2, const Vertex2D& intersection) const {
            return ComputeAngle(l1, intersection) < ComputeAngle(l2, intersection);
        }

        Float ComputeAngle(const LineSegment2D& l, const Vertex2D& intersection) const {
            if (AreEqual(LowerVertex(l), intersection, Epsilon) && math::IsZero(l[0][1] - l[1][1], Epsilon)) {
                return 2 * math::PI;
            }
            const auto v = AreEqual(LowerVertex(l), intersection, Epsilon) ? UpperVertex(l) : LowerVertex(l);
            const Vertex2D sweepLineVertex{-1, 0};
            const Vertex2D segmentVertex{(v - intersection).normalized()};
            return Angle360(sweepLineVertex, segmentVertex);
        }

    private:
        Vertex2D EventPoint;
        Float Epsilon;
    };
} // namespace compg
