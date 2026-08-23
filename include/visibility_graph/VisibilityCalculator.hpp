#pragma once

#include "math/primitives/Polygon.hpp"

namespace compg {
    class VisibilityCalculator {
    public:
        explicit VisibilityCalculator(Float epsilon = EPSILON)
            : Epsilon(epsilon) {}
        [[nodiscard]] std::vector<Vertex2D>
        FindVisibleVertices(const Vertex2D& origin, const std::vector<Polygon>& polygons) const;

    private:
        Float Epsilon;
    };
} // namespace compg