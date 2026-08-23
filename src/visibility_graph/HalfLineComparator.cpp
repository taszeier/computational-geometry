#include "visibility_graph/HalfLineComparator.hpp"
#include "math/Geometry.hpp"

namespace compg {
    bool HalfLineComparator::operator()(const LineSegment2D& lhs, const LineSegment2D& rhs) const {
        const auto lhsDistance = FindDistance(lhs);
        const auto rhsDistance = FindDistance(rhs);
        return math::IsZero(lhsDistance - rhsDistance, Epsilon) ? FindAngle(lhs) < FindAngle(rhs)
                                                                : lhsDistance < rhsDistance;
    }

    bool HalfLineComparator::operator()(const LineSegment2D& segment, const Vertex2D& vertex) const {
        const auto distance1 = FindDistance(segment);
        const auto distance2 = (HalfLine.GetOrigin() - vertex).norm();
        return distance1 + Epsilon < distance2;
    }

    bool HalfLineComparator::operator()(const Vertex2D& vertex, const LineSegment2D& segment) const {
        const auto distance1 = (HalfLine.GetOrigin() - vertex).norm();
        const auto distance2 = FindDistance(segment);
        return distance1 + Epsilon < distance2;
    }

    Float HalfLineComparator::FindDistance(const LineSegment2D& segment) const {
        const auto intersection = FindIntersection(HalfLine, segment, Epsilon);
        COMPG_ASSERT(intersection, "Expected the half line to intersect the line segment");
        return (HalfLine.GetOrigin() - intersection.value()).norm();
    }

    Float HalfLineComparator::FindAngle(const LineSegment2D& segment) const {
        const auto v0 = HalfLine.GetOrigin();
        const auto v1 = v0 + HalfLine.GetNormal();
        const auto side0 = FindSide(v0, v1, segment[0], Epsilon);
        const auto side1 = FindSide(v0, v1, segment[1], Epsilon);

        COMPG_ASSERT(
            !(side0 == PointSide::Collinear && side1 == PointSide::Collinear), "Expected a non-collinear line segment"
        );
        COMPG_ASSERT(
            side0 == PointSide::Collinear || side1 == PointSide::Collinear,
            "Expected at least one endpoint to be on the line"
        );
        const Vertex2D v = side0 == PointSide::Collinear ? segment[1] - segment[0] : segment[0] - segment[1];
        return Angle360(v0 - v1, v);
    }
} // namespace compg