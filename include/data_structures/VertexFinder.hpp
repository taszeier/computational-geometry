#pragma once

#include "common/Vertex.hpp"
#include "range_query/kd_tree/BallQueryRegion.hpp"
#include "range_query/kd_tree/KdTree.hpp"
#include "range_query/kd_tree/KdTreeQuery.hpp"

namespace compg {
    template <std::size_t K>
    struct VertexFinder {
        explicit VertexFinder(const std::vector<Vertex<K>>& vertices)
            : Tree{vertices} {}

        std::vector<std::size_t> Find(const Vertex<K>& query, Float epsilon = EPSILON) const {
            constexpr KdTreeQuery<K> treeQuery;
            const auto queryResult = treeQuery.Query(Tree, BallQueryRegion<K>({query, epsilon}));
            return queryResult | std::views::transform([](const VertexRecord<K>& record) { return record.Index; })
                   | std::ranges::to<std::vector>();
        }

        KdTree<K> Tree;
    };
} // namespace compg