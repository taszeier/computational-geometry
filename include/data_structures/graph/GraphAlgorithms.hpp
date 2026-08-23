#pragma once

#include <optional>
#include <vector>

#include "common/Error.hpp"
#include "data_structures/PriorityQueue.hpp"

namespace compg {
    template <typename NodeType>
    std::vector<NodeType> BacktrackPath(const NodeType& start, const NodeType& goal, const auto& predecessor) {
        auto current = goal;

        std::vector<NodeType> path;
        while (current != start) {
            path.push_back(current);
            current = predecessor.at(current);
        }

        path.push_back(start);
        std::ranges::reverse(path);
        return path;
    }

    class DijkstrasAlgorithm {
    public:
        template <typename GraphType>
        std::optional<std::vector<typename GraphType::node_type>> FindShortestPath(
            const GraphType& graph, const typename GraphType::node_type& start,
            const typename GraphType::node_type& goal
        ) const {
            COMPG_ASSERT(graph.HasNode(start), "Expected the start node to be in the graph");
            COMPG_ASSERT(graph.HasNode(goal), "Expected the goal node to be in the graph");

            using node_type = GraphType::node_type;
            using weight_type = GraphType::weight_type;
            using edge_type = GraphType::edge_type;

            PriorityQueue<std::tuple<node_type, node_type, weight_type>, decltype([](const auto& t1, const auto& t2) {
                              return std::get<2>(t1) > std::get<2>(t2);
                          })>
                queue;
            queue.Insert(std::tuple{start, start, static_cast<weight_type>(0)});
            std::unordered_map<node_type, node_type> predecessor;

            while (!queue.IsEmpty()) {
                const auto [previousNode, currentNode, cost] = queue.Top();
                queue.Pop();

                if (!predecessor.contains(currentNode)) {
                    predecessor[currentNode] = previousNode;
                    if (currentNode == goal) {
                        return BacktrackPath(start, goal, predecessor);
                    }

                    for (const auto& adjacentNode : graph.GetAdjacentNodes(currentNode)) {
                        if (!predecessor.contains(adjacentNode)) {
                            const auto weight = graph.GetWeight(edge_type{currentNode, adjacentNode});
                            queue.Insert(std::tuple{currentNode, adjacentNode, cost + weight});
                        }
                    }
                }
            }

            return std::nullopt;
        }
    };
} // namespace compg
