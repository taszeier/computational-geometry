#include "window_query/PrioritySearchTree.hpp"

#include "common/Algorithms.hpp"

namespace compg {
    namespace details {
        std::unique_ptr<PrioritySearchTreeNode>
        CreatePrioritySearchTree(std::vector<VertexRecord<2UZ>>& vertexRecords) {
            if (vertexRecords.size() == 0) {
                return nullptr;
            }
            if (vertexRecords.size() == 1) {
                return std::make_unique<PrioritySearchTreeNode>(vertexRecords.at(0), vertexRecords.at(0).Value);
            }

            const auto leftMostIt
                = std::ranges::min_element(vertexRecords, RotatedLexicographicalLess<2UZ>(0), ValueProjector{});
            const auto leftMost = *leftMostIt;
            vertexRecords.erase(leftMostIt);
            const auto medianIt = PartitionMedian(vertexRecords, RotatedLexicographicalLess<2UZ>(1), ValueProjector{});
            const auto median = medianIt->Value;

            auto leftNodes
                = std::ranges::subrange(vertexRecords.begin(), medianIt + 1) | std::ranges::to<std::vector>();
            auto leftChild = CreatePrioritySearchTree(leftNodes);
            auto rightNodes = std::ranges::subrange(medianIt + 1, vertexRecords.end()) | std::ranges::to<std::vector>();
            auto rightChild = CreatePrioritySearchTree(rightNodes);

            return std::make_unique<PrioritySearchTreeNode>(
                leftMost, median, std::move(leftChild), std::move(rightChild)
            );
        }

        void ReportInSubtree(
            PrioritySearchTree::node_type* nodePtr, Float queryLocation, std::vector<VertexRecord<2UZ>>& output
        ) {
            if (nodePtr != nullptr && nodePtr->Record.Value[0] <= queryLocation) {
                output.push_back(nodePtr->Record);
                ReportInSubtree(nodePtr->LeftChild.get(), queryLocation, output);
                ReportInSubtree(nodePtr->RightChild.get(), queryLocation, output);
            }
        }

        bool IsInsideQueryRegion(const VertexRecord<2UZ>& record, const SegmentQueryRegion& queryRegion) {
            return record.Value[0] <= queryRegion.Location && queryRegion.Interval.Lower <= record.Value[1]
                   && record.Value[1] <= queryRegion.Interval.Upper;
        };

        PrioritySearchTree::node_type* HandleSplitNodeSearch(
            PrioritySearchTree::node_type* rootPtr, const SegmentQueryRegion& queryRegion, auto medianComparator,
            auto& result
        ) {
            const Vertex2D lower{queryRegion.Location, queryRegion.Interval.Lower};
            const Vertex2D upper{queryRegion.Location, queryRegion.Interval.Upper};

            auto nodePtr = rootPtr;
            while (nodePtr != nullptr
                   && (medianComparator(upper, nodePtr->Median) || medianComparator(nodePtr->Median, lower))) {
                if (IsInsideQueryRegion(nodePtr->Record, queryRegion)) {
                    result.push_back(nodePtr->Record);
                }
                if (medianComparator(upper, nodePtr->Median)) {
                    nodePtr = nodePtr->LeftChild.get();
                } else {
                    nodePtr = nodePtr->RightChild.get();
                }
            }

            if (nodePtr != nullptr && details::IsInsideQueryRegion(nodePtr->Record, queryRegion)) {
                result.push_back(nodePtr->Record);
            }

            return nodePtr;
        }

        void HandleLeftSide(
            PrioritySearchTree::node_type* nodePtr, const SegmentQueryRegion& queryRegion, auto medianComparator,
            auto& result
        ) {
            COMPG_ASSERT(nodePtr != nullptr, "Expected a valid pointer");

            const Vertex2D lower{queryRegion.Location, queryRegion.Interval.Lower};
            auto leftPtr = nodePtr->LeftChild.get();
            while (leftPtr != nullptr) {
                if (IsInsideQueryRegion(leftPtr->Record, queryRegion)) {
                    result.push_back(leftPtr->Record);
                }
                if (medianComparator(leftPtr->Median, lower)) {
                    details::ReportInSubtree(leftPtr->RightChild.get(), queryRegion.Location, result);
                    leftPtr = leftPtr->LeftChild.get();
                } else {
                    leftPtr = leftPtr->RightChild.get();
                }
            }
        }

        void HandleRightSide(
            PrioritySearchTree::node_type* nodePtr, const SegmentQueryRegion& queryRegion, auto medianComparator,
            auto& result
        ) {
            COMPG_ASSERT(nodePtr != nullptr, "Expected a valid pointer");

            const Vertex2D upper{queryRegion.Location, queryRegion.Interval.Upper};
            auto rightPtr = nodePtr->RightChild.get();
            while (rightPtr != nullptr) {
                if (IsInsideQueryRegion(rightPtr->Record, queryRegion)) {
                    result.push_back(rightPtr->Record);
                }
                if (medianComparator(upper, rightPtr->Median)) {
                    details::ReportInSubtree(rightPtr->LeftChild.get(), queryRegion.Location, result);
                    rightPtr = rightPtr->RightChild.get();
                } else {
                    rightPtr = rightPtr->LeftChild.get();
                }
            }
        }
    } // namespace details

    PrioritySearchTree::PrioritySearchTree(const std::vector<Vertex2D>& vertices) {
        auto vertexRecords = vertices | std::views::enumerate | std::views::transform([](const auto& tup) {
                                 const auto& [index, vertex] = tup;
                                 return VertexRecord<2UZ>{static_cast<std::size_t>(index), vertex};
                             })
                             | std::ranges::to<std::vector>();
        Root = details::CreatePrioritySearchTree(vertexRecords);
    }

    std::vector<VertexRecord<2UZ>> PrioritySearchTree::Query(const SegmentQueryRegion& queryRegion) const {
        std::vector<VertexRecord<2UZ>> result;

        const RotatedLexicographicalLess<2UZ> medianComparator{1};
        const auto nodePtr = details::HandleSplitNodeSearch(Root.get(), queryRegion, medianComparator, result);
        if (nodePtr != nullptr) {
            details::HandleLeftSide(nodePtr, queryRegion, medianComparator, result);
            details::HandleRightSide(nodePtr, queryRegion, medianComparator, result);
        }

        return result;
    }
} // namespace compg
