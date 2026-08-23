#pragma once
#include "common/Vertex.hpp"
#include "data_structures/DisjointSetUnion.hpp"
#include "range_query/kd_tree/KdTree.hpp"

namespace compg {
    class VertexEquivalenceCalculator {
    public:
        using tree_type = KdTree<2UZ>;

        explicit VertexEquivalenceCalculator(Float epsilon = EPSILON)
            : Epsilon(epsilon) {}
        [[nodiscard]] DisjointSetUnion FindEquivalenceClasses(const std::vector<Vertex2D>& vertices) const;

    private:
        [[nodiscard]] std::vector<std::size_t>
        FindEquivalentVertices(const Vertex2D& query, const tree_type& tree) const;

        Float Epsilon;
    };
} // namespace compg
