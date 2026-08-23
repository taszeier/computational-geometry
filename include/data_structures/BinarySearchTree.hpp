#pragma once

#include "common/Common.hpp"
#include <memory>
#include <ranges>

namespace compg {
    template <typename ValueType>
    struct BinaryTreeNode {
        [[nodiscard]] constexpr bool IsLeaf() const {
            return LeftChild == nullptr && RightChild == nullptr;
        }

        ValueType Value;
        std::unique_ptr<BinaryTreeNode> LeftChild = nullptr;
        std::unique_ptr<BinaryTreeNode> RightChild = nullptr;
    };

    namespace details {
        template <typename ValueType>
        using node_type = BinaryTreeNode<ValueType>;

        template <typename ValueType>
        using pointer_type = std::unique_ptr<node_type<ValueType>>;

        template <typename ValueType, std::forward_iterator IteratorType>
        pointer_type<ValueType> CreateBalancedNode(IteratorType begin, IteratorType end) {
            if (begin == end) {
                return nullptr;
            }
            const auto size = std::distance(begin, end);
            const auto middle = std::next(begin, (size - 1) / 2);
            auto node = std::make_unique<node_type<ValueType>>(*middle);
            if (size > 1) {
                node->LeftChild = CreateBalancedNode<ValueType>(begin, std::next(middle));
                node->RightChild = CreateBalancedNode<ValueType>(std::next(middle), end);
            }

            return node;
        }
    } // namespace details

    template <typename ValueType, typename ComparatorType = Less, typename ProjectorType = Identity>
    class BinarySearchTree {
    public:
        using node_type = BinaryTreeNode<ValueType>;
        using pointer_type = std::unique_ptr<node_type>;

    public:
        explicit BinarySearchTree(const ComparatorType& comparator = {}, const ProjectorType& projector = {})
            : Comparator(comparator)
            , Projector(projector) {}

        const pointer_type& GetRoot() const {
            return Root;
        }
        bool Insert(const ValueType& value) {
            if (Root == nullptr) {
                Root = std::make_unique<node_type>(value);
                return true;
            }
            pointer_type* nodePtr = &Root;
            const auto valueProjected = Projector(value);
            while (!(*nodePtr)->IsLeaf()) {
                const auto nodeProjected = Projector((*nodePtr)->Value);
                if (Comparator(valueProjected, nodeProjected)) {
                    nodePtr = &(*nodePtr)->LeftChild;
                } else if (Comparator(nodeProjected, valueProjected)) {
                    nodePtr = &(*nodePtr)->RightChild;
                } else {
                    return false;
                }
            }
            const auto nodeProjected = Projector((*nodePtr)->Value);
            if (Comparator(valueProjected, nodeProjected)) {
                (*nodePtr)->LeftChild = std::make_unique<node_type>(value);
                (*nodePtr)->RightChild = std::make_unique<node_type>((*nodePtr)->Value);
                return true;
            }
            if (Comparator(nodeProjected, valueProjected)) {
                (*nodePtr)->LeftChild = std::make_unique<node_type>((*nodePtr)->Value);
                (*nodePtr)->RightChild = std::make_unique<node_type>(value);
                return true;
            }
            return false;
        }

        template <std::ranges::range Range>
        void Insert(const Range& range) {
            for (auto& element : range) {
                Insert(element);
            }
        }

        void Insert(std::initializer_list<ValueType> list) {
            for (const auto& element : list) {
                Insert(element);
            }
        }

        template <std::ranges::sized_range RangeType>
        static auto CreateBalanced(RangeType&& range, ComparatorType comparator = {}, ProjectorType projector = {}) {
            std::ranges::sort(range, comparator, projector);
            std::ranges::for_each(
                range | std::views::transform(projector) | std::views::slide(2), [comparator](const auto& pair) {
                    const auto it1 = pair.begin();
                    const auto it2 = std::next(it1);
                    COMPG_ASSERT(comparator(*it1, *it2), "Received values with identical projections in range");
                }
            );
            BinarySearchTree tree{comparator, projector};
            tree.Root = details::CreateBalancedNode<ValueType>(range.begin(), range.end());
            return tree;
        }

        const auto& GetComparator() const {
            return Comparator;
        }

        const auto& GetProjector() const {
            return Projector;
        }

    private:
        pointer_type Root = nullptr;
        ComparatorType Comparator;
        ProjectorType Projector;
    };
} // namespace compg
