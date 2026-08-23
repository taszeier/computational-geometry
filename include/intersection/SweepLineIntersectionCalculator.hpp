#pragma once

#include "common/Common.hpp"
#include "intersection/IntersectionCalculatorResult.hpp"
#include "math/Geometry.hpp"
#include <vector>

namespace compg {
    class SweepLineIntersectionCalculator {
    public:
        using segments_type = std::vector<LineSegment2D>;

    public:
        explicit SweepLineIntersectionCalculator(Float epsilon = EPSILON)
            : Epsilon(epsilon) {}

        /**
         * @brief Find intersections among a set of line segments.
         * @details The intersection of two line segments cannot contain more than one point.
         * @param segments A vector of unique line segments.
         * @return The IntersectionCalculatorResult.
         */
        [[nodiscard]] IntersectionCalculatorResult FindIntersections(const segments_type& segments) const;

    private:
        Float Epsilon;
    };
} // namespace compg