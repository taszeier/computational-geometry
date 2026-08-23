#pragma once

#include "common/Common.hpp"
#include "quad_tree/Directions.hpp"
#include "quad_tree/Quadrant.hpp"
#include <memory>
#include <queue>

namespace compg {

    template <typename QuadTreeType, typename NodeType>
    class QuadTreeMixin {
        using node_type = NodeType;
        using internal_node_type = node_type::internal_node_type;
        using leaf_node_type = node_type::leaf_node_type;

    public:
        void MakeBalanced() {
            auto leaves = GetLeaves();
            while (!leaves.empty()) {
                auto node = leaves.front();
                leaves.pop();

                if (RequiresSplit(node)) {
                    auto presplitNeighbors = GetAllCardinalDirections()
                                             | std::views::transform([node](auto d) { return node->FindNeighbor(d); })
                                             | std::ranges::to<std::vector>();

                    internal_node_type newState{};
                    for (auto direction : GetAllOrdinalDirections()) {
                        const auto leafBox = CreateQuadrant(node->Box, direction);
                        leaf_node_type leafNode
                            = static_cast<QuadTreeType*>(this)->CreateLeafNode(*node, leafBox, direction);
                        newState.GetChild(direction) = std::make_unique<node_type>(std::move(leafNode), leafBox, node);
                    }

                    for (auto& child : newState.Children) {
                        leaves.push(child.get());
                    }

                    node->State = std::move(newState);
                    for (auto neighbor : presplitNeighbors) {
                        if (neighbor != nullptr && RequiresSplit(neighbor)) {
                            leaves.push(neighbor);
                        }
                    }
                }
            }
        }

    private:
        constexpr bool RequiresSplit(node_type* node) {
            COMPG_ASSERT(node != nullptr, "Received invalid node");

            return node->IsLeaf() && std::ranges::any_of(GetAllCardinalDirections(), [node](auto direction) {
                       const auto neighbor = node->FindNeighbor(direction);
                       if (neighbor == nullptr)
                           return false;
                       return std::visit(
                           Overloads{
                               [direction](const internal_node_type& n) {
                                   return std::ranges::any_of(
                                       GetAllOrdinalDirections(), [&n, direction](auto ordinalDirection) {
                                           return HasComponent(ordinalDirection, Flip(direction))
                                                  && !n.GetChild(ordinalDirection)->IsLeaf();
                                       }
                                   );
                               },
                               [](const leaf_node_type&) { return false; }
                           },
                           neighbor->State
                       );
                   });
        }

        std::queue<node_type*> GetLeaves() const {
            std::queue<node_type*> leaves;
            const auto self = static_cast<const QuadTreeType*>(this);

            std::queue<node_type*> nodes{{self->GetRoot().get()}};
            while (!nodes.empty()) {
                const auto node = nodes.front();
                nodes.pop();

                std::visit(
                    Overloads{
                        [&nodes](const internal_node_type& n) {
                            for (const auto& child : n.Children) {
                                nodes.push(child.get());
                            }
                        },
                        [&leaves, node](const leaf_node_type&) { leaves.push(node); }
                    },
                    node->State
                );
            }

            return leaves;
        }
    };
} // namespace compg
