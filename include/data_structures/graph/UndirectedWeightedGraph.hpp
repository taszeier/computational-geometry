#pragma once

#include "common/Common.hpp"
#include "data_structures/graph/UndirectedEdge.hpp"
#include <unordered_map>
#include <unordered_set>

namespace compg {
    template <typename NodeType, typename ComparatorType = Less>
    class UndirectedWeightedGraph {
    public:
        using node_type = NodeType;
        using edge_type = UndirectedEdge<node_type, ComparatorType>;
        using node_set_type = std::unordered_set<node_type>;
        using weight_type = Float;

        constexpr bool HasNode(const node_type& node) const {
            return AdjacencyMap.contains(node);
        }

        constexpr bool HasEdge(const edge_type& edge) const {
            return EdgeWeights.contains(edge);
        }

        constexpr auto NumNodes() const {
            return AdjacencyMap.size();
        }

        bool InsertNode(const node_type& node) {
            if (!HasNode(node)) {
                AdjacencyMap.emplace(node, node_set_type{});
                return true;
            }
            return false;
        }

        void InsertEdge(const edge_type& edge, weight_type weight) {
            COMPG_ASSERT(HasNode(edge[0]) && HasNode(edge[1]), "Expected a valid edge");
            AdjacencyMap.at(edge[0]).insert(edge[1]);
            AdjacencyMap.at(edge[1]).insert(edge[0]);
            EdgeWeights[edge] = weight;
        }

        const auto& GetAdjacentNodes(const node_type& node) const {
            COMPG_ASSERT(HasNode(node), "Expected a valid node index");
            return AdjacencyMap.at(node);
        }

        weight_type GetWeight(const edge_type& edge) const {
            const auto it = EdgeWeights.find(edge);
            COMPG_ASSERT(it != EdgeWeights.end(), "Expected an edge in the graph.");
            return it->second;
        }

    private:
        std::unordered_map<node_type, node_set_type> AdjacencyMap;
        std::unordered_map<edge_type, weight_type> EdgeWeights;
    };
} // namespace compg