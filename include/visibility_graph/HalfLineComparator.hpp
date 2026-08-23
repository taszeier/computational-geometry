#pragma once

#include "math/primitives/HalfLine.hpp"
#include "math/primitives/LineSegment.hpp"

namespace compg {
    /**
     * @brief Compare line segments intersected by a half line using the distance from their intersections to the origin
     * of the half line.
     */
    class HalfLineComparator {
    public:
        explicit HalfLineComparator(const HalfLine2D& halfLine, Float epsilon = EPSILON)
            : HalfLine(halfLine)
            , Epsilon(epsilon) {}

        bool operator()(const LineSegment2D& lhs, const LineSegment2D& rhs) const;
        bool operator()(const LineSegment2D& segment, const Vertex2D& vertex) const;
        bool operator()(const Vertex2D& vertex, const LineSegment2D& segment) const;

    private:
        [[nodiscard]] Float FindDistance(const LineSegment2D& segment) const;
        [[nodiscard]] Float FindAngle(const LineSegment2D& segment) const;

    private:
        HalfLine2D HalfLine;
        Float Epsilon;
    };
} // namespace compg