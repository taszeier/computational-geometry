#pragma once

#include "common/Random.hpp"
#include "data_structures/graph/UndirectedWeightedGraph.hpp"
#include "math/primitives/Box.hpp"
#include "math/primitives/Polygon.hpp"
#include "motion_planning/FreeSpace.hpp"

namespace compg {
    class ShortestPointPathCalculator {
    public:
        using graph_type = UndirectedWeightedGraph<Vertex2D, LexicographicalLess<Vertex2D>>;

        ShortestPointPathCalculator(
            const Box2D& box, const std::vector<Polygon>& obstacles, std::size_t seed = DEFAULT_SEED
        );
        std::optional<std::vector<Vertex2D>> FindPath(const Vertex2D& start, const Vertex2D& goal) const;

    private:
        FreeSpace FreeSpace;
        std::vector<Polygon> Obstacles;
        graph_type VisibilityGraph;
    };
} // namespace compg
