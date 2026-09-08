#pragma once
#include <ranges>
#include <utility>

namespace compg {

    template <typename GraphType>
    concept Graph = requires(GraphType graph) {
        typename GraphType::node_type;
        typename GraphType::edge_type;
        { graph.HasNode(std::declval<typename GraphType::node_type>()) } -> std::same_as<bool>;
        { graph.HasEdge(std::declval<typename GraphType::edge_type>()) } -> std::same_as<bool>;
        { graph.GetAdjacentNodes(std::declval<typename GraphType::node_type>()) } -> std::ranges::range;
    };

    template <typename GraphType>
    concept WeightedGraph = requires(GraphType graph) {
        requires Graph<GraphType>;
        typename GraphType::weight_type;
        {
            graph.GetWeight(std::declval<typename GraphType::edge_type>())
        } -> std::same_as<typename GraphType::weight_type>;
    };
} // namespace compg