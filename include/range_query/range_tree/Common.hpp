#pragma once

#include "common/Common.hpp"

namespace compg {
    template <
        std::size_t VertexDim, std::size_t TreeDim, template <std::size_t, std::size_t> typename TreeType,
        template <std::size_t> typename QueryRegionType>
    const TreeType<VertexDim, TreeDim>::pointer_type& FindSplitNode(
        const typename TreeType<VertexDim, TreeDim>::pointer_type& root, const QueryRegionType<VertexDim>& queryRegion
    ) {
        constexpr auto coordinateIndex = TreeType<VertexDim, TreeDim>::CoordinateIndex;
        auto nodePtr = &root;
        bool foundSplit = false;

        while (!foundSplit) {
            foundSplit = std::visit(
                Overloads{
                    [&nodePtr, &queryRegion](const typename TreeType<VertexDim, TreeDim>::internal_node_type& n) {
                        if (queryRegion.GetUpperCorner()(coordinateIndex) < n.Median(coordinateIndex)) {
                            nodePtr = &n.LeftChild;
                            return false;
                        }
                        if (queryRegion.GetLowerCorner()(coordinateIndex) > n.Median(coordinateIndex)) {
                            nodePtr = &n.RightChild;
                            return false;
                        }
                        return true;
                    },
                    [](const typename TreeType<VertexDim, TreeDim>::leaf_node_type&) { return true; }
                },
                (*nodePtr)->State
            );
        }
        return *nodePtr;
    }
} // namespace compg