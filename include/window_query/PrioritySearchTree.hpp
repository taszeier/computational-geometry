#pragma once

#include "common/Vertex.hpp"
#include "window_query/SegmentQueryRegion.hpp"
#include <memory>

namespace compg {
    struct PrioritySearchTreeNode {
        VertexRecord<2UZ> Record;
        Vertex2D Median;
        std::unique_ptr<PrioritySearchTreeNode> LeftChild = nullptr;
        std::unique_ptr<PrioritySearchTreeNode> RightChild = nullptr;
    };

    class PrioritySearchTree {
    public:
        using node_type = PrioritySearchTreeNode;
        using pointer_type = std::unique_ptr<node_type>;

        explicit PrioritySearchTree(const std::vector<Vertex2D>& vertices);

        std::vector<VertexRecord<2UZ>> Query(const SegmentQueryRegion& queryRegion) const;

    private:
        explicit PrioritySearchTree(pointer_type root)
            : Root(std::move(root)) {}

    private:
        pointer_type Root = nullptr;
    };
} // namespace compg
