#pragma once

#include "convex_hull/ConvexHull.hpp"

namespace compg {
    class AndrewsMonotoneChain final : public ConvexHullCalculator<2UZ> {
    public:
        using vertex_type = Vertex2D;

    public:
        explicit AndrewsMonotoneChain(Float epsilon = EPSILON)
            : Epsilon(epsilon) {}
        [[nodiscard]] convex_hull_type FindConvexHull(const std::vector<vertex_type>& vertices) const override;

    private:
        Float Epsilon;
    };
} // namespace compg