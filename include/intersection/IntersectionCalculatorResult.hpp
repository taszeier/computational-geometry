#pragma once

#include "common/Vertex.hpp"

#include <unordered_set>
#include <vector>

namespace compg {
    /**
     * @brief The result of the intersection calculator.
     * @details Contains the set of intersection points and the intersecting segments for each intersection.
     */
    struct IntersectionCalculatorResult {
        std::vector<Vertex2D> Intersections;
        std::vector<std::unordered_set<std::size_t>> IntersectingSegments;
    };

    /**
     * @brief Combine intersection points that are close to each other.
     * @details This function is useful when the same intersection point is detected multiple times due to numerical
     * error. The sets of intersecting segments of duplicate intersections are unioned.
     * @param result The result of the intersection calculator.
     * @param epsilon An error threshold used to find duplicate intersections.
     * @return The intersection calculator result with no duplicate intersections.
     */
    IntersectionCalculatorResult Collapse(const IntersectionCalculatorResult& result, Float epsilon = EPSILON);
} // namespace compg