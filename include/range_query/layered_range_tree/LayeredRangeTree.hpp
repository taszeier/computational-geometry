#pragma once

#include "common/Algorithms.hpp"
#include "common/Vertex.hpp"
#include <algorithm>
#include <memory>

namespace compg {
    template <typename ValueType, typename PointerType>
    struct FractionalCascade {
        std::vector<ValueType> Values;
        std::vector<PointerType> LeftChildPointers;
        std::vector<PointerType> RightChildPointers;

        constexpr auto Size() const {
            return Values.size();
        }
    };

    template <std::size_t VertexDim>
    struct LayeredRangeTreeLeafNode {
        VertexRecord<VertexDim> Record;
    };

    template <std::size_t VertexDim, std::size_t TreeDim>
    struct LayeredRangeTreeNode;

    template <std::size_t VertexDim, std::size_t TreeDim>
    class LayeredRangeTree;

    template <std::size_t VertexDim, std::size_t TreeDim>
    struct LayeredRangeTreeInternalNode {
        Vertex<VertexDim> Median;
        std::unique_ptr<LayeredRangeTreeNode<VertexDim, TreeDim>> LeftChild;
        std::unique_ptr<LayeredRangeTreeNode<VertexDim, TreeDim>> RightChild;
        LayeredRangeTree<VertexDim, TreeDim - 1> AssociatedStructure;
    };

    template <std::size_t VertexDim>
    struct LayeredRangeTreeInternalNode<VertexDim, 2UZ> {
        Vertex<VertexDim> Median;
        std::unique_ptr<LayeredRangeTreeNode<VertexDim, 2UZ>> LeftChild;
        std::unique_ptr<LayeredRangeTreeNode<VertexDim, 2UZ>> RightChild;
        FractionalCascade<VertexRecord<VertexDim>, std::size_t> AssociatedStructure;
    };

    template <std::size_t VertexDim, std::size_t TreeDim>
    struct LayeredRangeTreeNode {
        std::variant<LayeredRangeTreeInternalNode<VertexDim, TreeDim>, LayeredRangeTreeLeafNode<VertexDim>> State;

        [[nodiscard]] constexpr bool IsLeaf() const {
            return State.index() == 1UZ;
        }
    };

    namespace layered_range_tree::details {
        template <typename ValueType, typename ComparatorType = Less, typename ProjectorType = Identity>
        auto CreateFractionalCascade(
            const std::vector<ValueType>& values, const std::vector<ValueType>& leftChildValues,
            const std::vector<ValueType>& rightChildValues, ComparatorType comparator = {}, ProjectorType projector = {}
        ) {

            auto leftChildPointers = BisectLeftMany(leftChildValues, values, comparator, projector);
            auto rightChildPointers = BisectLeftMany(rightChildValues, values, comparator, projector);
            return FractionalCascade<ValueType, std::size_t>{
                values, std::move(leftChildPointers), std::move(rightChildPointers)
            };
        }

        template <
            std::size_t VertexDim, std::size_t TreeDim, typename ComparatorType = Less,
            typename ProjectorType = Identity, std::ranges::range RangeType>
        auto CreateFractionalCascade(
            RangeType&& vertices, const LayeredRangeTreeNode<VertexDim, TreeDim>& leftChild,
            const LayeredRangeTreeNode<VertexDim, TreeDim>& rightChild, ComparatorType comparator = {},
            ProjectorType projector = {}
        ) {
            std::vector<std::ranges::range_value_t<RangeType>> empty;
            const auto& leftChildVertices
                = leftChild.IsLeaf() ? empty : std::get<0>(leftChild.State).AssociatedStructure.Values;
            const auto& rightChildVertices
                = rightChild.IsLeaf() ? empty : std::get<0>(rightChild.State).AssociatedStructure.Values;

            return CreateFractionalCascade(
                vertices | std::ranges::to<std::vector>(), leftChildVertices, rightChildVertices, comparator, projector
            );
        }
    } // namespace layered_range_tree::details

    namespace layered_range_tree::details {
        template <std::size_t VertexDim, std::size_t TreeDim, std::ranges::sized_range RangeType>
            requires(std::same_as<std::ranges::range_value_t<RangeType>, VertexRecord<VertexDim>>)
        std::unique_ptr<LayeredRangeTreeNode<VertexDim, TreeDim>> CreateNode(RangeType&& vertexRecords);

        template <std::size_t VertexDim, std::size_t TreeDim, std::ranges::sized_range RangeType, typename FunctionType>
        auto CreateInternalNode(RangeType&& vertexRecords, FunctionType&& structureCreator) {

            constexpr auto coordinateIndex = VertexDim - TreeDim;
            const auto medianIterator = PartitionMedian(
                vertexRecords, RotatedLexicographicalLess<VertexDim>(coordinateIndex), ValueProjector{}
            );
            const auto median = medianIterator->Value;

            auto leftChild
                = CreateNode<VertexDim, TreeDim>(std::ranges::subrange(vertexRecords.begin(), medianIterator + 1));
            auto rightChild
                = CreateNode<VertexDim, TreeDim>(std::ranges::subrange(medianIterator + 1, vertexRecords.end()));

            auto structure = structureCreator(vertexRecords, *leftChild, *rightChild);
            return std::make_unique<LayeredRangeTreeNode<VertexDim, TreeDim>>(
                LayeredRangeTreeInternalNode<VertexDim, TreeDim>{
                    median, std::move(leftChild), std::move(rightChild), std::move(structure)
                }
            );
        }

        template <std::size_t VertexDim, std::size_t TreeDim, std::ranges::sized_range RangeType>
            requires(std::same_as<std::ranges::range_value_t<RangeType>, VertexRecord<VertexDim>>)
        std::unique_ptr<LayeredRangeTreeNode<VertexDim, TreeDim>> CreateNode(RangeType&& vertexRecords) {
            COMPG_ASSERT(!vertexRecords.empty(), "Cannot create a LayeredRangeTree from zero vertices");

            if (vertexRecords.size() == 1) {
                return std::make_unique<LayeredRangeTreeNode<VertexDim, TreeDim>>(
                    LayeredRangeTreeLeafNode<VertexDim>(*vertexRecords.begin())
                );
            }

            if constexpr (TreeDim > 2) {
                auto structureCreator = [](auto&& vertices, const LayeredRangeTreeNode<VertexDim, TreeDim>&,
                                           const LayeredRangeTreeNode<VertexDim, TreeDim>&) {
                    return LayeredRangeTree<VertexDim, TreeDim - 1>::Create(vertices);
                };
                return CreateInternalNode<VertexDim, TreeDim>(std::forward<RangeType>(vertexRecords), structureCreator);
            } else {
                auto structureCreator = [](auto&& vertices, const LayeredRangeTreeNode<VertexDim, TreeDim>& leftChild,
                                           const LayeredRangeTreeNode<VertexDim, TreeDim>& rightChild) {
                    std::ranges::sort(vertices, RotatedLexicographicalLess<VertexDim>(VertexDim - 1), ValueProjector{});
                    return CreateFractionalCascade<VertexDim, TreeDim>(
                        vertices, leftChild, rightChild, RotatedLexicographicalLess<VertexDim>{VertexDim - 1},
                        ValueProjector{}
                    );
                };
                return CreateInternalNode<VertexDim, TreeDim>(std::forward<RangeType>(vertexRecords), structureCreator);
            }
        }

    } // namespace layered_range_tree::details

    template <std::size_t VertexDim, std::size_t TreeDim = VertexDim>
    class LayeredRangeTree {
        static_assert(VertexDim > 1, "One-dimensional vertices are not supported");
        static_assert(TreeDim != 0, "The tree cannot be zero dimensional");
        static_assert(
            TreeDim <= VertexDim, "The dimension of the tree cannot be greater "
                                  "than the dimension of the vertices"
        );

    public:
        using node_type = LayeredRangeTreeNode<VertexDim, TreeDim>;
        using internal_node_type = LayeredRangeTreeInternalNode<VertexDim, TreeDim>;
        using leaf_node_type = LayeredRangeTreeLeafNode<VertexDim>;
        using pointer_type = std::unique_ptr<node_type>;

        static constexpr std::size_t CoordinateIndex = VertexDim - TreeDim;

        explicit LayeredRangeTree(const std::vector<Vertex<VertexDim>>& vertices) {
            COMPG_ASSERT(!vertices.empty(), "Cannot create a LayeredRangeTree from zero vertices");
            auto vertexRecords = vertices | std::views::enumerate | std::views::transform([](const auto& tup) {
                                     const auto& [index, value] = tup;
                                     return VertexRecord<VertexDim>{static_cast<std::size_t>(index), value};
                                 })
                                 | std::ranges::to<std::vector>();
            Root = layered_range_tree::details::CreateNode<VertexDim, TreeDim>(vertexRecords);
        }

        template <std::ranges::sized_range RangeType>
            requires(std::is_same_v<std::ranges::range_value_t<RangeType>, Vertex<VertexDim>>)
        static auto Create(RangeType&& vertices) {
            COMPG_ASSERT(!vertices.empty(), "Cannot create a LayeredRangeTree from zero vertices");
            auto vertexRecords = vertices | std::views::enumerate | std::views::transform([](const auto& tup) {
                                     const auto& [index, value] = tup;
                                     return VertexRecord<VertexDim>{index, value};
                                 })
                                 | std::ranges::to<std::vector>();
            LayeredRangeTree tree{};
            tree.Root = layered_range_tree::details::CreateNode<VertexDim, TreeDim>(vertexRecords);
            return tree;
        }

        template <std::ranges::sized_range RangeType>
            requires(std::is_same_v<std::ranges::range_value_t<RangeType>, VertexRecord<VertexDim>>)
        static auto Create(RangeType&& vertexRecords) {
            COMPG_ASSERT(!vertexRecords.empty(), "Cannot create a LayeredRangeTree from zero vertices");
            LayeredRangeTree tree{};
            tree.Root
                = layered_range_tree::details::CreateNode<VertexDim, TreeDim>(std::forward<RangeType>(vertexRecords));
            return tree;
        }

        const pointer_type& GetRoot() const {
            return Root;
        }

    private:
        LayeredRangeTree() = default;

        pointer_type Root = nullptr;
    };
} // namespace compg