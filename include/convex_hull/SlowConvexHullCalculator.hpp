#pragma once

#include "convex_hull/ConvexHull.hpp"

namespace compg {
    class SlowConvexHullCalculator final : public ConvexHullCalculator<2UZ> {
    public:
        using vertex_type = Vertex2D;

    public:
        explicit SlowConvexHullCalculator(Float epsilon = EPSILON)
            : Epsilon(epsilon) {}
        [[nodiscard]] convex_hull_type FindConvexHull(const std::vector<vertex_type>& vertices) const override;

    private:
        [[nodiscard]] bool
        IsValid(const vertex_type& v1, const vertex_type& v2, const std::vector<vertex_type>& vertices) const;

    private:
        Float Epsilon;
    };
} // namespace compg