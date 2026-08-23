#pragma once

#include "common/Common.hpp"
#include "math/primitives/Box.hpp"
#include "quad_tree/Directions.hpp"
#include <memory>
#include <queue>
#include <ranges>

#include "quad_tree/QuadTreeMixin.hpp"
#include "quad_tree/QuadTreeNodeMixin.hpp"

namespace compg {
    struct QuadTreeNode;

    struct QuadTreeInternalNode {
        QuadTreeInternalNode() = default;
        QuadTreeInternalNode(const QuadTreeInternalNode&);
        QuadTreeInternalNode& operator=(const QuadTreeInternalNode&);
        QuadTreeInternalNode(QuadTreeInternalNode&&) = default;
        QuadTreeInternalNode& operator=(QuadTreeInternalNode&&) = default;
        ~QuadTreeInternalNode() = default;

        std::unique_ptr<QuadTreeNode>& GetChild(OrdinalDirection d) {
            return Children.at(static_cast<std::size_t>(d));
        }

        [[nodiscard]] const std::unique_ptr<QuadTreeNode>& GetChild(OrdinalDirection d) const {
            return Children.at(static_cast<std::size_t>(d));
        }

        std::array<std::unique_ptr<QuadTreeNode>, 4> Children;
    };

    struct QuadTreeLeafNode {
        std::optional<Vertex2D> MaybeVertex;
    };

    struct QuadTreeNode : QuadTreeNodeMixin<QuadTreeNode, QuadTreeInternalNode, QuadTreeLeafNode> {
        using internal_node_type = QuadTreeInternalNode;
        using leaf_node_type = QuadTreeLeafNode;
        using state_type = std::variant<internal_node_type, leaf_node_type>;

        QuadTreeNode(state_type state, const Box2D& box, QuadTreeNode* parent)
            : State(std::move(state))
            , Box(box)
            , Parent(parent) {}

        [[nodiscard]] constexpr bool IsLeaf() const {
            return State.index() == 1UZ;
        }

        state_type State;
        Box2D Box;
        QuadTreeNode* Parent = nullptr;
    };

    class QuadTree : public QuadTreeMixin<QuadTree, QuadTreeNode> {
    public:
        using node_type = QuadTreeNode;
        using internal_node_type = node_type::internal_node_type;
        using leaf_node_type = node_type::leaf_node_type;

        explicit QuadTree(const std::vector<Vertex2D>& vertices, const Box2D& box);
        QuadTree(const QuadTree& t);
        QuadTree& operator=(const QuadTree& t);
        QuadTree(QuadTree&&) = default;
        QuadTree& operator=(QuadTree&&) = default;
        ~QuadTree() = default;

        [[nodiscard]] const std::unique_ptr<QuadTreeNode>& GetRoot() const {
            return Root;
        }

        static leaf_node_type
        CreateLeafNode(const node_type& presplitNode, const Box2D&, OrdinalDirection leafDirection);

    private:
        std::unique_ptr<QuadTreeNode> Root;
    };
} // namespace compg
