#pragma once
#include "math/primitives/LineSegment.hpp"

namespace compg {
    /**
     * @brief Get the left and right endpoints of a line segment.
     * @param segment A line segment.
     * @return The left and right endpoints.
     */
    inline auto GetEndPoints(const LineSegment2D& segment) {
        auto v0 = segment[0];
        auto v1 = segment[1];
        if (!LexicographicalLess<Vertex2D>::Compare(v0, v1)) {
            std::swap(v0, v1);
        }

        return std::tuple{v0, v1};
    }
} // namespace compg
