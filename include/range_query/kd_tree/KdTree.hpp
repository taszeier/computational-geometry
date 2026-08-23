#pragma once

#include "common/Algorithms.hpp"
#include "common/Common.hpp"
#include "math/primitives/Hyperplane.hpp"

#include <memory>

namespace compg {
    template <std::size_t K>
    struct KdTreeNode;

    template <std::size_t K>
    struct KdTreeInternalNode {
        AxesAlignedHyperplane<K> Plane;
        std::unique_ptr<KdTreeNode<K>> LeftChild;
        std::unique_ptr<KdTreeNode<K>> RightChild;
    };

    template <std::size_t K>
    struct KdTreeLeafNode {
        VertexRecord<K> Record;
    };

    template <std::size_t K>
    struct KdTreeNode {
        std::variant<KdTreeInternalNode<K>, KdTreeLeafNode<K>> State;

        [[nodiscard]] constexpr bool IsLeaf() const {
            return State.index() == 1UZ;
        }
    };

    namespace details {
        template <std::size_t K, std::ranges::range VertexRange>
            requires std::same_as<std::ranges::range_value_t<VertexRange>, VertexRecord<K>>
        std::unique_ptr<KdTreeNode<K>> CreateKdTree(VertexRange&& vertexRecords, std::size_t depth) {
            COMPG_ASSERT(!vertexRecords.empty(), "Cannot create a KdTree from no vertices");
            if (vertexRecords.size() == 1) {
                return std::make_unique<KdTreeNode<K>>(KdTreeLeafNode<K>(*vertexRecords.begin()));
            }
            const auto coordinateIndex = depth % K;
            const auto medianIterator
                = PartitionMedian(vertexRecords, RotatedLexicographicalLess<K>(coordinateIndex), ValueProjector{});

            AxesAlignedHyperplane<K> plane{coordinateIndex, medianIterator->Value[coordinateIndex]};
            auto leftChild
                = CreateKdTree<K>(std::ranges::subrange(vertexRecords.begin(), medianIterator + 1), depth + 1);
            auto rightChild
                = CreateKdTree<K>(std::ranges::subrange(medianIterator + 1, vertexRecords.end()), depth + 1);

            return std::make_unique<KdTreeNode<K>>(
                KdTreeInternalNode<K>{std::move(plane), std::move(leftChild), std::move(rightChild)}
            );
        }
    } // namespace details

    template <std::size_t K>
    class KdTree {
    public:
        using node_type = KdTreeNode<K>;
        using pointer_type = std::unique_ptr<node_type>;

        explicit KdTree(const std::vector<Vertex<K>>& vertices) {
            COMPG_ASSERT(!vertices.empty(), "Cannot create a KdTree from zero vertices");
            auto vertexRecords = vertices | std::views::enumerate | std::views::transform([](const auto& tup) {
                                     const auto& [index, value] = tup;
                                     return VertexRecord<K>{static_cast<std::size_t>(index), value};
                                 })
                                 | std::ranges::to<std::vector>();
            Root = details::CreateKdTree<K>(vertexRecords, 0);
        }

        const pointer_type& GetRoot() const {
            return Root;
        }

    private:
        pointer_type Root;
    };
} // namespace compg
