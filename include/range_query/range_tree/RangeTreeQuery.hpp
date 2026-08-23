#pragma once

#include "range_query/one_dim/RangeQuery1D.hpp"
#include "range_query/range_tree/Common.hpp"
#include "range_query/range_tree/RangeTree.hpp"
#include "range_query/range_tree/RangeTreeQueryRegion.hpp"

namespace compg {
    template <std::size_t VertexDim, std::size_t TreeDim>
    class RangeTreeQuery;

    namespace details {
        template <std::size_t VertexDim>
        using output_container_type = std::vector<VertexRecord<VertexDim>>;

        template <std::size_t VertexDim>
        void HandleLeaf(
            const RangeTreeLeafNode<VertexDim>& leaf, const RangeTreeQueryRegion<VertexDim>& queryRegion,
            output_container_type<VertexDim>& output
        ) {
            if (queryRegion.Contains(leaf.Record.Value)) {
                output.push_back(leaf.Record);
            }
        }

        template <std::size_t VertexDim, std::size_t TreeDim>
        void HandleLeftSide(
            const RangeTreeInternalNode<VertexDim, TreeDim>& node, const RangeTreeQueryRegion<VertexDim>& queryRegion,
            output_container_type<VertexDim>& output
        ) {
            constexpr auto coordinateIndex = RangeTree<VertexDim, TreeDim>::CoordinateIndex;
            auto nodePtr = &node.LeftChild;
            bool foundLeaf = false;
            while (!foundLeaf) {
                foundLeaf = std::visit(
                    Overloads{
                        [&nodePtr, &queryRegion, &output](const RangeTreeInternalNode<VertexDim, TreeDim>& node) {
                            if (queryRegion.GetLowerCorner()(coordinateIndex) <= node.Median(coordinateIndex)) {
                                if constexpr (TreeDim > 2) {
                                    RangeTreeQuery<VertexDim, TreeDim - 1> query;
                                    query.Query(node.RightChild->AssociatedStructure, queryRegion, output);
                                } else {
                                    RangeQuery1D query{};
                                    Range1D<Float> range{
                                        queryRegion.GetLowerCorner()(VertexDim - 1),
                                        queryRegion.GetUpperCorner()(VertexDim - 1)
                                    };
                                    query.Query(node.RightChild->AssociatedStructure, range, output);
                                }

                                nodePtr = &node.LeftChild;
                            } else {
                                nodePtr = &node.RightChild;
                            }
                            return false;
                        },
                        [&queryRegion, &output](const RangeTreeLeafNode<VertexDim>& leaf) {
                            details::HandleLeaf<VertexDim>(leaf, queryRegion, output);
                            return true;
                        }
                    },
                    (*nodePtr)->State
                );
            }
        }

        template <std::size_t VertexDim, std::size_t TreeDim>
        void HandleRightSide(
            const RangeTreeInternalNode<VertexDim, TreeDim>& node, const RangeTreeQueryRegion<VertexDim>& queryRegion,
            output_container_type<VertexDim>& output
        ) {
            constexpr auto coordinateIndex = RangeTree<VertexDim, TreeDim>::CoordinateIndex;
            auto nodePtr = &node.RightChild;
            bool foundLeaf = false;
            while (!foundLeaf) {
                foundLeaf = std::visit(
                    Overloads{
                        [&nodePtr, &queryRegion, &output](const RangeTreeInternalNode<VertexDim, TreeDim>& node) {
                            if (node.Median(coordinateIndex) <= queryRegion.GetUpperCorner()(coordinateIndex)) {
                                if constexpr (TreeDim > 2) {
                                    RangeTreeQuery<VertexDim, TreeDim - 1> query;
                                    query.Query(node.LeftChild->AssociatedStructure, queryRegion, output);
                                } else {
                                    RangeQuery1D query{};
                                    Range1D<Float> range{
                                        queryRegion.GetLowerCorner()(VertexDim - 1),
                                        queryRegion.GetUpperCorner()(VertexDim - 1)
                                    };
                                    query.Query(node.LeftChild->AssociatedStructure, range, output);
                                }

                                nodePtr = &node.RightChild;
                            } else {
                                nodePtr = &node.LeftChild;
                            }
                            return false;
                        },
                        [&queryRegion, &output](const RangeTreeLeafNode<VertexDim>& leaf) {
                            details::HandleLeaf<VertexDim>(leaf, queryRegion, output);
                            return true;
                        }
                    },
                    (*nodePtr)->State
                );
            }
        }
    } // namespace details

    template <std::size_t VertexDim, std::size_t TreeDim = VertexDim>
    class RangeTreeQuery {
    public:
        using pointer_type = RangeTree<VertexDim, TreeDim>::pointer_type;
        using output_container_type = std::vector<VertexRecord<VertexDim>>;

        output_container_type
        Query(const RangeTree<VertexDim, TreeDim>& tree, const RangeTreeQueryRegion<VertexDim>& queryRegion) const {
            output_container_type result{};
            Query(tree.GetRoot(), queryRegion, result);
            return result;
        }

        void Query(
            const RangeTree<VertexDim, TreeDim>& tree, const RangeTreeQueryRegion<VertexDim>& queryRegion,
            output_container_type& output
        ) const {
            Query(tree.GetRoot(), queryRegion, output);
        }

    private:
        void Query(
            const pointer_type& node, const RangeTreeQueryRegion<VertexDim>& queryRegion, output_container_type& output
        ) const {
            const auto& splitNode = FindSplitNode<VertexDim, TreeDim, RangeTree>(node, queryRegion);
            std::visit(
                Overloads{
                    [&queryRegion, &output](const RangeTreeLeafNode<VertexDim>& leaf) {
                        details::HandleLeaf<VertexDim>(leaf, queryRegion, output);
                    },
                    [&queryRegion, &output](const RangeTreeInternalNode<VertexDim, TreeDim>& node) {
                        details::HandleLeftSide<VertexDim, TreeDim>(node, queryRegion, output);
                        details::HandleRightSide<VertexDim, TreeDim>(node, queryRegion, output);
                    }
                },
                splitNode->State
            );
        }
    };
} // namespace compg