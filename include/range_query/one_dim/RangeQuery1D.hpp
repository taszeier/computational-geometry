#pragma once

#include "data_structures/BinarySearchTree.hpp"
#include "range_query/one_dim/Range1D.hpp"

namespace compg {
    namespace details {
        template <typename TreeValueType>
        void
        ReportSubtree(const std::unique_ptr<BinaryTreeNode<TreeValueType>>& node, std::vector<TreeValueType>& output) {
            COMPG_ASSERT(node != nullptr, "Expected a non-empty subtree");
            if (node->IsLeaf()) {
                output.push_back(node->Value);
            } else {
                ReportSubtree(node->LeftChild, output);
                ReportSubtree(node->RightChild, output);
            }
        }

        template <typename TreeValueType, typename ComparatorType, typename ProjectorType, typename RangeValueType>
        void HandleLeaf(
            const std::unique_ptr<BinaryTreeNode<TreeValueType>>& leaf, const ComparatorType& comparator,
            const ProjectorType& projector, const Range1D<RangeValueType>& range, std::vector<TreeValueType>& output
        ) {
            COMPG_ASSERT(leaf != nullptr && leaf->IsLeaf(), "Expected a pointer to a leaf node");
            const auto projection = projector(leaf->Value);
            if (!comparator(projection, range.Lower) && !comparator(range.Upper, projection)) {
                output.push_back(leaf->Value);
            }
        };

        template <typename TreeValueType, typename ComparatorType, typename ProjectorType, typename RangeValueType>
        void HandleLeftSide(
            const std::unique_ptr<BinaryTreeNode<TreeValueType>>& splitNode, const ComparatorType& comparator,
            const ProjectorType& projector, const Range1D<RangeValueType>& range, std::vector<TreeValueType>& output
        ) {
            COMPG_ASSERT(splitNode != nullptr, "The split node cannot be null");
            auto nodePtr = &splitNode->LeftChild;
            while (!(*nodePtr)->IsLeaf()) {
                if (!comparator(projector((*nodePtr)->Value), range.Lower)) {
                    ReportSubtree((*nodePtr)->RightChild, output);
                    nodePtr = &(*nodePtr)->LeftChild;
                } else {
                    nodePtr = &(*nodePtr)->RightChild;
                    ;
                }
                COMPG_ASSERT((*nodePtr) != nullptr, "Only leaf nodes should have null children");
            }
            HandleLeaf(*nodePtr, comparator, projector, range, output);
        }

        template <typename TreeValueType, typename ComparatorType, typename ProjectorType, typename RangeValueType>
        void HandleRightSide(
            const std::unique_ptr<BinaryTreeNode<TreeValueType>>& splitNode, const ComparatorType& comparator,
            const ProjectorType& projector, const Range1D<RangeValueType>& range, std::vector<TreeValueType>& output
        ) {
            COMPG_ASSERT(splitNode != nullptr, "The split node cannot be null");
            auto nodePtr = &splitNode->RightChild;
            while (!(*nodePtr)->IsLeaf()) {
                if (!comparator(range.Upper, projector((*nodePtr)->Value))) {
                    ReportSubtree((*nodePtr)->LeftChild, output);
                    nodePtr = &(*nodePtr)->RightChild;
                } else {
                    nodePtr = &(*nodePtr)->LeftChild;
                    ;
                }
                COMPG_ASSERT((*nodePtr) != nullptr, "Only leaf nodes should have null children");
            }
            HandleLeaf(*nodePtr, comparator, projector, range, output);
        }
    } // namespace details

    class RangeQuery1D {
    public:
        template <typename TreeValueType, typename ComparatorType, typename ProjectorType, typename RangeValueType>
        void Query(
            const BinarySearchTree<TreeValueType, ComparatorType, ProjectorType>& tree,
            const Range1D<RangeValueType>& range, std::vector<TreeValueType>& output
        ) const {
            if (tree.GetRoot() == nullptr) {
                return;
            }

            const auto& splitNode = FindSplitNode(tree, range);
            COMPG_ASSERT(splitNode != nullptr, "The split node cannot be null");
            if (splitNode->IsLeaf()) {
                details::HandleLeaf(splitNode, tree.GetComparator(), tree.GetProjector(), range, output);
            } else {
                details::HandleLeftSide(splitNode, tree.GetComparator(), tree.GetProjector(), range, output);
                details::HandleRightSide(splitNode, tree.GetComparator(), tree.GetProjector(), range, output);
            }
        }

        template <typename TreeValueType, typename ComparatorType, typename ProjectorType, typename RangeValueType>
        auto Query(
            const BinarySearchTree<TreeValueType, ComparatorType, ProjectorType>& tree,
            const Range1D<RangeValueType>& range
        ) const {
            std::vector<TreeValueType> result;
            Query(tree, range, result);
            return result;
        }

    private:
        template <typename TreeValueType, typename ComparatorType, typename ProjectorType, typename RangeValueType>
        const std::unique_ptr<BinaryTreeNode<TreeValueType>>& FindSplitNode(
            const BinarySearchTree<TreeValueType, ComparatorType, ProjectorType>& tree,
            const Range1D<RangeValueType>& range
        ) const {
            auto nodePtr = &tree.GetRoot();
            COMPG_ASSERT((*nodePtr) != nullptr, "Received an empty tree");
            const auto& projector = tree.GetProjector();
            const auto& comparator = tree.GetComparator();

            auto projection = projector((*nodePtr)->Value);
            while (!(*nodePtr)->IsLeaf()
                   && (!comparator(projection, range.Upper) || comparator(projection, range.Lower))) {
                if (!comparator(projection, range.Upper)) {
                    nodePtr = &(*nodePtr)->LeftChild;
                } else {
                    nodePtr = &(*nodePtr)->RightChild;
                }
                COMPG_ASSERT((*nodePtr) != nullptr, "Only leaf nodes should have null children");
                projection = projector((*nodePtr)->Value);
            }

            return *nodePtr;
        }
    };
} // namespace compg