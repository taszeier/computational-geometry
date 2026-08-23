#include "quad_tree/QuadTree.hpp"
// #include "math/Geometry.hpp"
#include "quad_tree/Quadrant.hpp"

namespace compg {
    namespace details {
        template <std::ranges::sized_range Vertex2DRange>
            requires std::same_as<std::ranges::range_value_t<Vertex2DRange>, Vertex2D>
        std::unique_ptr<QuadTreeNode>
        CreateTree(QuadTreeNode* parent, const Box2D& box, const Vertex2DRange& vertices) {
            if (vertices.size() >= 2) {
                auto node = std::make_unique<QuadTreeNode>(QuadTreeInternalNode{}, box, parent);
                const auto center = box.GetCenter();

                QuadTreeInternalNode state;
                for (auto direction : GetAllOrdinalDirections()) {
                    const auto childBox = CreateQuadrant(box, direction);
                    state.GetChild(direction) = CreateTree(
                        node.get(), childBox, vertices | std::views::filter([&center, direction](const auto& v) {
                                                  return direction == FindQuadrantOfVertex(center, v);
                                              }) | std::ranges::to<std::vector>()
                    );
                }
                node->State = std::move(state);
                return node;
            }
            auto maybeVertex = vertices.size() ? std::optional<Vertex2D>{*vertices.begin()} : std::nullopt;
            return std::make_unique<QuadTreeNode>(QuadTreeLeafNode{std::move(maybeVertex)}, box, parent);
        }
    } // namespace details

    QuadTree::QuadTree(const std::vector<Vertex2D>& vertices, const Box2D& box) {
        std::ranges::for_each(vertices, [&box](const auto& v) {
            COMPG_ASSERT(box.Contains(v), "Expected the box to contain the vertices");
        });
        Root = details::CreateTree(nullptr, box, vertices);
    }

    QuadTree::QuadTree(const QuadTree& t)
        : Root(std::make_unique<QuadTreeNode>(*t.Root)) {}

    QuadTree& QuadTree::operator=(const QuadTree& t) {
        if (this != &t) {
            Root = std::make_unique<QuadTreeNode>(*t.Root);
        }
        return *this;
    }

    QuadTree::leaf_node_type
    QuadTree::CreateLeafNode(const node_type& presplitNode, const Box2D&, OrdinalDirection leafDirection) {
        const leaf_node_type& presplitLeafNode = std::get<1>(presplitNode.State);
        auto maybeVertex = presplitLeafNode.MaybeVertex.and_then(
            [&presplitNode, leafDirection](const Vertex2D& v) -> std::optional<Vertex2D> {
                const auto vertexDirection = FindQuadrantOfVertex(presplitNode.Box.GetCenter(), v);
                if (vertexDirection == leafDirection) {
                    return v;
                }
                return std::nullopt;
            }
        );
        return {std::move(maybeVertex)};
    }

    QuadTreeInternalNode::QuadTreeInternalNode(const QuadTreeInternalNode& n) {
        for (std::size_t i = 0; i < n.Children.size(); ++i) {
            Children[i] = std::make_unique<QuadTreeNode>(*n.Children[i]);
        }
    }

    QuadTreeInternalNode& QuadTreeInternalNode::operator=(const QuadTreeInternalNode& n) {
        if (this != &n) {
            for (std::size_t i = 0; i < n.Children.size(); ++i) {
                Children[i] = std::make_unique<QuadTreeNode>(*n.Children[i]);
            }
        }
        return *this;
    }
} // namespace compg
