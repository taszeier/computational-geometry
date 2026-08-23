#pragma once

#include "quad_tree/Directions.hpp"
#include <memory>

#include "data_structures/DoublyConnectedEdgeList.hpp"
#include "math/primitives/Box.hpp"
#include "quad_tree/CircuitSegment.hpp"
#include "quad_tree/QuadTreeMixin.hpp"
#include "quad_tree/QuadTreeNodeMixin.hpp"

namespace compg {
    struct CircuitQuadTreeNode;

    struct CircuitQuadTreeInternalNode {
        std::unique_ptr<CircuitQuadTreeNode>& GetChild(OrdinalDirection d) {
            return Children.at(static_cast<std::size_t>(d));
        }

        [[nodiscard]] const std::unique_ptr<CircuitQuadTreeNode>& GetChild(OrdinalDirection d) const {
            return Children.at(static_cast<std::size_t>(d));
        }

        std::array<std::unique_ptr<CircuitQuadTreeNode>, 4> Children;
    };

    struct CircuitQuadTreeLeafNode {
        std::optional<bool> IsInteriorSegmentAngle45;
    };

    struct CircuitQuadTreeNode
        : QuadTreeNodeMixin<CircuitQuadTreeNode, CircuitQuadTreeInternalNode, CircuitQuadTreeLeafNode> {
        using internal_node_type = CircuitQuadTreeInternalNode;
        using leaf_node_type = CircuitQuadTreeLeafNode;
        using state_type = std::variant<internal_node_type, leaf_node_type>;

        CircuitQuadTreeNode(state_type state, const Box2D& box, CircuitQuadTreeNode* parent)
            : State(std::move(state))
            , Box(box)
            , Parent(parent) {}

        [[nodiscard]] constexpr bool IsLeaf() const {
            return State.index() == 1UZ;
        }

        state_type State;
        Box2D Box;
        CircuitQuadTreeNode* Parent = nullptr;
    };

    class CircuitQuadTree : public QuadTreeMixin<CircuitQuadTree, CircuitQuadTreeNode> {
    public:
        using node_type = CircuitQuadTreeNode;
        using internal_node_type = node_type::internal_node_type;
        using leaf_node_type = node_type::leaf_node_type;

        explicit CircuitQuadTree(const std::vector<CircuitSegment>& segments, std::size_t power);

    private:
        explicit CircuitQuadTree(const std::vector<CircuitSegment>& segments, const Box2D& box);

    public:
        [[nodiscard]] DoublyConnectedEdgeList CreateMesh();
        [[nodiscard]] const std::unique_ptr<CircuitQuadTreeNode>& GetRoot() const {
            return Root;
        }

        static leaf_node_type
        CreateLeafNode(const node_type& presplitNode, const Box2D&, OrdinalDirection leafDirection);

    private:
        std::unique_ptr<CircuitQuadTreeNode> Root;
    };
} // namespace compg
