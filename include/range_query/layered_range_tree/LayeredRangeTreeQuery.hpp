#pragma once

#include "range_query/layered_range_tree/LayeredRangeTree.hpp"
#include "range_query/layered_range_tree/LayeredRangeTreeQueryRegion.hpp"
#include "range_query/range_tree/Common.hpp"

namespace compg {
    template <std::size_t VertexDim, std::size_t TreeDim>
    class LayeredRangeTreeQuery;

    namespace details {
        template <std::size_t VertexDim>
        using output_container_type = std::vector<VertexRecord<VertexDim>>;

        template <std::size_t VertexDim>
        void HandleLeaf(
            const LayeredRangeTreeLeafNode<VertexDim>& leaf, const RangeTreeQueryRegion<VertexDim>& queryRegion,
            output_container_type<VertexDim>& output
        ) {
            if (queryRegion.Contains(leaf.Record.Value)) {
                output.push_back(leaf.Record);
            }
        }

        template <std::size_t VertexDim, std::size_t TreeDim>
        void HandleNode(
            const LayeredRangeTreeNode<VertexDim, TreeDim>& node, const RangeTreeQueryRegion<VertexDim>& queryRegion,
            output_container_type<VertexDim>& output
        ) {
            std::visit(
                Overloads{
                    [&queryRegion, &output](const LayeredRangeTreeLeafNode<VertexDim>& leaf) {
                        HandleLeaf<VertexDim>(leaf, queryRegion, output);
                    },
                    [&queryRegion, &output](const LayeredRangeTreeInternalNode<VertexDim, TreeDim>& internalNode) {
                        LayeredRangeTreeQuery<VertexDim, TreeDim - 1> query;
                        query.Query(internalNode.AssociatedStructure, queryRegion, output);
                    }
                },
                node.State
            );
        }

        template <std::size_t VertexDim>
        void Report(
            const LayeredRangeTreeNode<VertexDim, 2UZ>& node, const RangeTreeQueryRegion<VertexDim>& queryRegion,
            std::size_t startIndex, output_container_type<VertexDim>& output
        ) {
            constexpr auto coordinateIndex = VertexDim - 1;

            std::visit(
                Overloads{
                    [&queryRegion, &output](const LayeredRangeTreeLeafNode<VertexDim>& leaf) {
                        HandleLeaf<VertexDim>(leaf, queryRegion, output);
                    },
                    [startIndex, &output,
                     &queryRegion](const LayeredRangeTreeInternalNode<VertexDim, 2UZ>& internalNode) {
                        auto notTooLarge = [&queryRegion](const VertexRecord<VertexDim>& v) {
                            return v.Value(coordinateIndex) <= queryRegion.GetUpperCorner()(coordinateIndex);
                        };
                        std::ranges::for_each(
                            internalNode.AssociatedStructure.Values | std::views::drop(startIndex)
                                | std::views::take_while(notTooLarge),
                            [&output](const VertexRecord<VertexDim>& v) { output.push_back(v); }
                        );
                    }
                },
                node.State
            );
        }

        template <std::size_t VertexDim, std::size_t TreeDim>
            requires(TreeDim == 2UZ)
        void HandleLeftSide(
            const LayeredRangeTreeInternalNode<VertexDim, TreeDim>& splitNode,
            const RangeTreeQueryRegion<VertexDim>& queryRegion, std::size_t splitNodeBisectIndex,
            output_container_type<VertexDim>& output
        ) {
            constexpr auto coordinateIndex = LayeredRangeTree<VertexDim, TreeDim>::CoordinateIndex;

            if (splitNodeBisectIndex >= splitNode.AssociatedStructure.Values.size()) {
                return;
            }

            auto nodePtr = &splitNode.LeftChild;
            auto bisectIndex = splitNode.AssociatedStructure.LeftChildPointers.at(splitNodeBisectIndex);

            bool done = false;
            while (!done) {
                done = std::visit(
                    Overloads{
                        [&nodePtr, &queryRegion, &output,
                         &bisectIndex](const LayeredRangeTreeInternalNode<VertexDim, TreeDim>& node) {
                            if (bisectIndex >= node.AssociatedStructure.Values.size()) {
                                return true;
                            }

                            if (queryRegion.GetLowerCorner()(coordinateIndex) <= node.Median(coordinateIndex)) {
                                Report<VertexDim>(
                                    *node.RightChild, queryRegion,
                                    node.AssociatedStructure.RightChildPointers.at(bisectIndex), output
                                );
                                nodePtr = &node.LeftChild;
                                bisectIndex = node.AssociatedStructure.LeftChildPointers.at(bisectIndex);
                            } else {
                                nodePtr = &node.RightChild;
                                bisectIndex = node.AssociatedStructure.RightChildPointers.at(bisectIndex);
                            }
                            return false;
                        },
                        [&queryRegion, &output](const LayeredRangeTreeLeafNode<VertexDim>& leaf) {
                            details::HandleLeaf<VertexDim>(leaf, queryRegion, output);
                            return true;
                        }
                    },
                    (*nodePtr)->State
                );
            }
        }

        template <std::size_t VertexDim, std::size_t TreeDim>
        void HandleLeftSide(
            const LayeredRangeTreeInternalNode<VertexDim, TreeDim>& node,
            const RangeTreeQueryRegion<VertexDim>& queryRegion, output_container_type<VertexDim>& output
        ) {
            constexpr auto coordinateIndex = LayeredRangeTree<VertexDim, TreeDim>::CoordinateIndex;
            auto nodePtr = &node.LeftChild;
            bool foundLeaf = false;
            while (!foundLeaf) {
                foundLeaf = std::visit(
                    Overloads{
                        [&nodePtr, &queryRegion,
                         &output](const LayeredRangeTreeInternalNode<VertexDim, TreeDim>& node) {
                            if (queryRegion.GetLowerCorner()(coordinateIndex) <= node.Median(coordinateIndex)) {
                                HandleNode<VertexDim, TreeDim>(*node.RightChild, queryRegion, output);
                                nodePtr = &node.LeftChild;
                            } else {
                                nodePtr = &node.RightChild;
                            }
                            return false;
                        },
                        [&queryRegion, &output](const LayeredRangeTreeLeafNode<VertexDim>& leaf) {
                            details::HandleLeaf<VertexDim>(leaf, queryRegion, output);
                            return true;
                        }
                    },
                    (*nodePtr)->State
                );
            }
        }

        template <std::size_t VertexDim, std::size_t TreeDim>
            requires(TreeDim == 2UZ)
        void HandleRightSide(
            const LayeredRangeTreeInternalNode<VertexDim, TreeDim>& splitNode,
            const RangeTreeQueryRegion<VertexDim>& queryRegion, std::size_t splitNodeBisectIndex,
            output_container_type<VertexDim>& output
        ) {
            constexpr auto coordinateIndex = LayeredRangeTree<VertexDim, TreeDim>::CoordinateIndex;

            if (splitNodeBisectIndex >= splitNode.AssociatedStructure.Values.size()) {
                return;
            }

            auto nodePtr = &splitNode.RightChild;
            auto bisectIndex = splitNode.AssociatedStructure.RightChildPointers.at(splitNodeBisectIndex);

            bool done = false;
            while (!done) {
                done = std::visit(
                    Overloads{
                        [&nodePtr, &queryRegion, &output,
                         &bisectIndex](const LayeredRangeTreeInternalNode<VertexDim, TreeDim>& node) {
                            if (bisectIndex >= node.AssociatedStructure.Values.size()) {
                                return true;
                            }

                            if (node.Median(coordinateIndex) <= queryRegion.GetUpperCorner()(coordinateIndex)) {
                                Report<VertexDim>(
                                    *node.LeftChild, queryRegion,
                                    node.AssociatedStructure.LeftChildPointers.at(bisectIndex), output
                                );
                                nodePtr = &node.RightChild;
                                bisectIndex = node.AssociatedStructure.RightChildPointers.at(bisectIndex);
                            } else {
                                nodePtr = &node.LeftChild;
                                bisectIndex = node.AssociatedStructure.LeftChildPointers.at(bisectIndex);
                            }
                            return false;
                        },
                        [&queryRegion, &output](const LayeredRangeTreeLeafNode<VertexDim>& leaf) {
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
            const LayeredRangeTreeInternalNode<VertexDim, TreeDim>& node,
            const RangeTreeQueryRegion<VertexDim>& queryRegion, output_container_type<VertexDim>& output
        ) {
            constexpr auto coordinateIndex = LayeredRangeTree<VertexDim, TreeDim>::CoordinateIndex;
            auto nodePtr = &node.RightChild;
            bool foundLeaf = false;
            while (!foundLeaf) {
                foundLeaf = std::visit(
                    Overloads{
                        [&nodePtr, &queryRegion,
                         &output](const LayeredRangeTreeInternalNode<VertexDim, TreeDim>& node) {
                            if (node.Median(coordinateIndex) <= queryRegion.GetLowerCorner()(coordinateIndex)) {
                                HandleNode<VertexDim, TreeDim>(*node.LeftChild, queryRegion, output);
                                nodePtr = &node.RightChild;
                            } else {
                                nodePtr = &node.LeftChild;
                            }
                            return false;
                        },
                        [&queryRegion, &output](const LayeredRangeTreeLeafNode<VertexDim>& leaf) {
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
    class LayeredRangeTreeQuery {
    public:
        using pointer_type = LayeredRangeTree<VertexDim, TreeDim>::pointer_type;
        using output_container_type = std::vector<VertexRecord<VertexDim>>;

        output_container_type Query(
            const LayeredRangeTree<VertexDim, TreeDim>& tree, const LayeredRangeTreeQueryRegion<VertexDim>& queryRegion
        ) const {
            output_container_type result{};
            Query(tree.GetRoot(), queryRegion, result);
            return result;
        }

        void Query(
            const LayeredRangeTree<VertexDim, TreeDim>& tree, const LayeredRangeTreeQueryRegion<VertexDim>& queryRegion,
            output_container_type& output
        ) const {
            Query(tree.GetRoot(), queryRegion, output);
        }

    private:
        void Query(
            const pointer_type& node, const LayeredRangeTreeQueryRegion<VertexDim>& queryRegion,
            output_container_type& output
        ) const {
            const auto& splitNode = FindSplitNode<VertexDim, TreeDim, LayeredRangeTree>(node, queryRegion);
            std::visit(
                Overloads{
                    [&queryRegion, &output](const LayeredRangeTreeLeafNode<VertexDim>& leaf) {
                        details::HandleLeaf<VertexDim>(leaf, queryRegion, output);
                    },
                    [&queryRegion, &output](const LayeredRangeTreeInternalNode<VertexDim, TreeDim>& node) {
                        if constexpr (TreeDim > 2) {
                            details::HandleLeftSide<VertexDim, TreeDim>(node, queryRegion, output);
                            details::HandleRightSide<VertexDim, TreeDim>(node, queryRegion, output);
                        } else {
                            auto projector = [coordinateProjector = CoordinateProjector<VertexDim>{VertexDim - 1},
                                              valueProjector = ValueProjector{}](const auto& record) {
                                return coordinateProjector(valueProjector(record));
                            };
                            const auto bisectIndex = BisectLeft(
                                node.AssociatedStructure.Values, queryRegion.GetLowerCorner()(VertexDim - 1), Less{},
                                projector
                            );
                            details::HandleLeftSide<VertexDim, TreeDim>(node, queryRegion, bisectIndex, output);
                            details::HandleRightSide<VertexDim, TreeDim>(node, queryRegion, bisectIndex, output);
                        }
                    }
                },
                splitNode->State
            );
        }
    };
} // namespace compg