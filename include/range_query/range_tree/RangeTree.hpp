#pragma once

#include "common/Algorithms.hpp"
#include "common/Common.hpp"
#include "common/Vertex.hpp"
#include <memory>

#include "data_structures/BinarySearchTree.hpp"

namespace compg {

    template <std::size_t VertexDim>
    struct RangeTreeLeafNode {
        VertexRecord<VertexDim> Record;
    };

    template <std::size_t VertexDim, std::size_t TreeDim>
    struct RangeTreeNode;

    template <std::size_t VertexDim, std::size_t TreeDim>
    struct RangeTreeInternalNode {
        Vertex<VertexDim> Median;
        std::unique_ptr<RangeTreeNode<VertexDim, TreeDim>> LeftChild;
        std::unique_ptr<RangeTreeNode<VertexDim, TreeDim>> RightChild;
    };

    template <std::size_t VertexDim, std::size_t TreeDim>
    class RangeTree;

    template <std::size_t VertexDim, std::size_t TreeDim>
    struct RangeTreeNode {
        std::variant<RangeTreeInternalNode<VertexDim, TreeDim>, RangeTreeLeafNode<VertexDim>> State;
        RangeTree<VertexDim, TreeDim - 1> AssociatedStructure;

        [[nodiscard]] constexpr bool IsLeaf() const {
            return State.index() == 1UZ;
        }
    };

    template <std::size_t VertexDim>
    struct RangeTreeNode<VertexDim, 2UZ> {
        std::variant<RangeTreeInternalNode<VertexDim, 2UZ>, RangeTreeLeafNode<VertexDim>> State;
        BinarySearchTree<VertexRecord<VertexDim>, RotatedLexicographicalLess<VertexDim>, ValueProjector>
            AssociatedStructure;

        [[nodiscard]] constexpr bool IsLeaf() const {
            return State.index() == 1UZ;
        }
    };

    namespace details {
        template <std::size_t VertexDim, std::size_t TreeDim, std::ranges::sized_range RangeType>
            requires(std::same_as<std::ranges::range_value_t<RangeType>, VertexRecord<VertexDim>>)
        std::unique_ptr<RangeTreeNode<VertexDim, TreeDim>> CreateRangeTree(RangeType&& vertexRecords);

        template <
            std::size_t VertexDim, std::size_t TreeDim, std::ranges::sized_range RangeType,
            typename AssociatedStructureType>
        auto CreateRangeTree(RangeType&& vertexRecords, AssociatedStructureType&& associatedStructure) {
            if (vertexRecords.size() == 1) {
                return std::make_unique<RangeTreeNode<VertexDim, TreeDim>>(
                    RangeTreeLeafNode<VertexDim>(*vertexRecords.begin()),
                    std::forward<AssociatedStructureType>(associatedStructure)
                );
            }
            constexpr auto coordinateIndex = VertexDim - TreeDim;
            const auto medianIterator = PartitionMedian(
                vertexRecords, RotatedLexicographicalLess<VertexDim>(coordinateIndex), ValueProjector{}
            );
            const auto median = medianIterator->Value;

            auto leftChild
                = CreateRangeTree<VertexDim, TreeDim>(std::ranges::subrange(vertexRecords.begin(), medianIterator + 1));
            auto rightChild
                = CreateRangeTree<VertexDim, TreeDim>(std::ranges::subrange(medianIterator + 1, vertexRecords.end()));

            return std::make_unique<RangeTreeNode<VertexDim, TreeDim>>(
                RangeTreeInternalNode<VertexDim, TreeDim>{median, std::move(leftChild), std::move(rightChild)},
                std::forward<AssociatedStructureType>(associatedStructure)
            );
        }

        template <std::size_t VertexDim, std::size_t TreeDim, std::ranges::sized_range RangeType>
            requires(std::same_as<std::ranges::range_value_t<RangeType>, VertexRecord<VertexDim>>)
        std::unique_ptr<RangeTreeNode<VertexDim, TreeDim>> CreateRangeTree(RangeType&& vertexRecords) {
            COMPG_ASSERT(!vertexRecords.empty(), "Cannot create a RangeTree from zero vertices");

            if constexpr (TreeDim > 2) {
                auto associatedStructure = RangeTree<VertexDim, TreeDim - 1>::Create(vertexRecords);
                return CreateRangeTree<VertexDim, TreeDim>(
                    std::forward<RangeType>(vertexRecords), std::move(associatedStructure)
                );
            } else {
                auto associatedStructure
                    = BinarySearchTree<VertexRecord<VertexDim>, RotatedLexicographicalLess<VertexDim>, ValueProjector>::
                        CreateBalanced(vertexRecords, RotatedLexicographicalLess<VertexDim>{VertexDim - 1});
                return CreateRangeTree<VertexDim, TreeDim>(
                    std::forward<RangeType>(vertexRecords), std::move(associatedStructure)
                );
            }
        }

    } // namespace details

    template <std::size_t VertexDim, std::size_t TreeDim = VertexDim>
    class RangeTree {
        static_assert(VertexDim > 1, "One-dimensional vertices are not supported");
        static_assert(TreeDim != 0, "The tree cannot be zero dimensional");
        static_assert(
            TreeDim <= VertexDim, "The dimension of the tree cannot be greater than the dimension of the vertices"
        );

    public:
        using node_type = RangeTreeNode<VertexDim, TreeDim>;
        using internal_node_type = RangeTreeInternalNode<VertexDim, TreeDim>;
        using leaf_node_type = RangeTreeLeafNode<VertexDim>;
        using pointer_type = std::unique_ptr<node_type>;

        static constexpr std::size_t CoordinateIndex = VertexDim - TreeDim;

        explicit RangeTree(const std::vector<Vertex<VertexDim>>& vertices) {
            COMPG_ASSERT(!vertices.empty(), "Cannot create a RangeTree from zero vertices");
            auto vertexRecords = vertices | std::views::enumerate | std::views::transform([](const auto& tup) {
                                     const auto& [index, value] = tup;
                                     return VertexRecord<VertexDim>{static_cast<std::size_t>(index), value};
                                 })
                                 | std::ranges::to<std::vector>();
            Root = details::CreateRangeTree<VertexDim, TreeDim>(vertexRecords);
        }

        template <std::ranges::sized_range RangeType>
            requires(std::is_same_v<std::ranges::range_value_t<RangeType>, Vertex<VertexDim>>)
        static auto Create(RangeType&& vertices) {
            COMPG_ASSERT(!vertices.empty(), "Cannot create a RangeTree from zero vertices");
            RangeTree tree{};
            auto vertexRecords = vertices | std::views::enumerate | std::views::transform([](const auto& tup) {
                                     const auto& [index, value] = tup;
                                     return VertexRecord<VertexDim>{index, value};
                                 })
                                 | std::ranges::to<std::vector>();
            tree.Root = details::CreateRangeTree<VertexDim, TreeDim>(vertexRecords);
            return tree;
        }

        template <std::ranges::sized_range RangeType>
            requires(std::is_same_v<std::ranges::range_value_t<RangeType>, VertexRecord<VertexDim>>)
        static auto Create(RangeType&& vertexRecords) {
            COMPG_ASSERT(!vertexRecords.empty(), "Cannot create a RangeTree from zero vertices");
            RangeTree tree{};
            tree.Root = details::CreateRangeTree<VertexDim, TreeDim>(std::forward<RangeType>(vertexRecords));
            return tree;
        }

        const pointer_type& GetRoot() const {
            return Root;
        }

    private:
        RangeTree() = default;

    private:
        pointer_type Root = nullptr;
    };
} // namespace compg
