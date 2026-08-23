#pragma once

#include "visibility_graph/ShortestPointPathCalculator.hpp"

namespace compg {
    class ShortestConvexPathCalculator {
    public:
        ShortestConvexPathCalculator(
            const Polygon& convexPolygon, const Box2D& box, const std::vector<Polygon>& obstacles,
            std::size_t seed = DEFAULT_SEED
        );

        std::optional<std::vector<Vertex2D>> FindPath(const Vertex2D& start, const Vertex2D& goal) const;

    private:
        std::optional<ShortestPointPathCalculator> Calculator;
    };
} // namespace compg