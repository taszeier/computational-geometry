#pragma once

#include <unordered_map>
#include <unordered_set>

namespace compg {
    template <typename LeftNodeType, typename RightNodeType>
    class BipartiteGraph {
    public:
        using left_node_type = LeftNodeType;
        using right_node_type = RightNodeType;
        template <typename NodeType>
        using node_set_type = std::unordered_set<NodeType>;

        constexpr bool HasLeftNode(const left_node_type& node) const {
            return RightAdjacentNodes.contains(node);
        }

        constexpr bool HasRightNode(const right_node_type& node) const {
            return LeftAdjacentNodes.contains(node);
        }

        constexpr bool HasEdge(const left_node_type& leftNode, const right_node_type& rightNode) const {
            const auto it = RightAdjacentNodes.find(leftNode);
            return it != RightAdjacentNodes.end() && it->second.contains(rightNode);
        }

        bool InsertLeftNode(const left_node_type& node) {
            if (!HasLeftNode(node)) {
                RightAdjacentNodes.emplace(node, node_set_type<right_node_type>{});
                return true;
            }
            return false;
        }

        bool InsertRightNode(const right_node_type& node) {
            if (!HasRightNode(node)) {
                LeftAdjacentNodes.emplace(node, node_set_type<left_node_type>{});
                return true;
            }
            return false;
        }

        void InsertEdge(const left_node_type& leftNode, const right_node_type& rightNode) {
            COMPG_ASSERT(HasLeftNode(leftNode) && HasRightNode(rightNode), "Expected both nodes to be in the graph");
            RightAdjacentNodes.at(leftNode).insert(rightNode);
            LeftAdjacentNodes.at(rightNode).insert(leftNode);
        }

        const auto& GetRightAdjacentNodes(const left_node_type& node) const {
            COMPG_ASSERT(HasLeftNode(node), "Expected a node in the graph");
            return RightAdjacentNodes.at(node);
        }

        const auto& GetLeftAdjacentNodes(const right_node_type& node) const {
            COMPG_ASSERT(HasRightNode(node), "Expected a node in the graph");
            return LeftAdjacentNodes.at(node);
        }

        bool EraseLeftNode(const left_node_type& node) {
            if (!HasLeftNode(node)) {
                return false;
            }

            const auto adjacentNodes = RightAdjacentNodes.at(node);
            for (const auto& adjacentNode : adjacentNodes) {
                LeftAdjacentNodes.at(adjacentNode).erase(node);
            }
            RightAdjacentNodes.erase(node);
            return true;
        }

        bool EraseRightNode(const right_node_type& node) {
            if (!HasRightNode(node)) {
                return false;
            }

            const auto adjacentNodes = LeftAdjacentNodes.at(node);
            for (const auto& adjacentNode : adjacentNodes) {
                RightAdjacentNodes.at(adjacentNode).erase(node);
            }
            LeftAdjacentNodes.erase(node);
            return true;
        }

    private:
        std::unordered_map<right_node_type, node_set_type<left_node_type>> LeftAdjacentNodes;
        std::unordered_map<left_node_type, node_set_type<right_node_type>> RightAdjacentNodes;
    };
} // namespace compg