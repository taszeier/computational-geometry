#include "data_structures/VertexEquivalenceCalculator.hpp"
#include "range_query/kd_tree/BallQueryRegion.hpp"
#include "range_query/kd_tree/KdTreeQuery.hpp"

namespace compg {
    DisjointSetUnion VertexEquivalenceCalculator::FindEquivalenceClasses(const std::vector<Vertex2D>& vertices) const {
        DisjointSetUnion classes{vertices.size()};
        tree_type tree{vertices};

        std::ranges::for_each(vertices | std::views::enumerate, [&tree, this, &classes](const auto& tup) {
            const auto& [i, vertex] = tup;
            const auto equivalentVertices = FindEquivalentVertices(vertex, tree);
            std::ranges::for_each(equivalentVertices, [&classes, i](const auto j) { classes.Union(i, j); });
        });

        return classes;
    }

    std::vector<std::size_t>
    VertexEquivalenceCalculator::FindEquivalentVertices(const Vertex2D& query, const tree_type& tree) const {
        constexpr KdTreeQuery<2UZ> treeQuery;
        const auto queryResult = treeQuery.Query(tree, BallQueryRegion<2UZ>({query, Epsilon}));
        return queryResult | std::views::transform(IndexProjector{}) | std::ranges::to<std::vector>();
    }
} // namespace compg