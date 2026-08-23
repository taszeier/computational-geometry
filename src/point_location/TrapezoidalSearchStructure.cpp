#include "point_location/TrapezoidalSearchStructure.hpp"
#include "math/Geometry.hpp"
#include "point_location/Common.hpp"

namespace compg {
    using SearchResult = TrapezoidalSearchStructure::SearchResult;

    SearchResult TrapezoidalSearchStructure::Search(const Vertex2D& vertex) const {
        std::size_t nodePtr = Root;
        while (!Nodes.at(nodePtr).IsLeaf()) {
            auto& internalNode = std::get<1>(Nodes.at(nodePtr).State);
            const auto maybeSearchResult = std::visit(
                Overloads{
                    [&vertex, &internalNode, &nodePtr](const XNode& xNode) -> std::optional<SearchResult> {
                        if (AreEqual(vertex, xNode.Vertex)) {
                            return SearchResult{xNode.Vertex};
                        }
                        nodePtr = LexicographicalLess<Vertex2D>::Compare(vertex, xNode.Vertex)
                                      ? internalNode.LeftChild
                                      : internalNode.RightChild;
                        return std::nullopt;
                    },
                    [&vertex, &internalNode, &nodePtr](const YNode& yNode) -> std::optional<SearchResult> {
                        const auto [v0, v1] = GetEndPoints(yNode.Segment);
                        const auto side = FindSide(v0, v1, vertex);
                        if (side == PointSide::Negative) {
                            nodePtr = internalNode.LeftChild;
                        } else if (side == PointSide::Positive) {
                            nodePtr = internalNode.RightChild;
                        } else if (side == PointSide::Collinear) {
                            if (AreEqual(yNode.Segment[0], vertex)) {
                                return SearchResult{yNode.Segment[0]};
                            }
                            if (AreEqual(yNode.Segment[1], vertex)) {
                                return SearchResult{yNode.Segment[1]};
                            }
                            return SearchResult{yNode.Segment};
                        } else {
                            COMPG_THROW("Received unexpected value");
                        }
                        return std::nullopt;
                    }
                },
                internalNode.Node
            );
            if (maybeSearchResult.has_value()) {
                return maybeSearchResult.value();
            }
        }

        return SearchResult{std::get<0>(Nodes.at(nodePtr).State).TrapezoidIndex};
    }

    TrapezoidalSearchStructure::trapezoid_index
    TrapezoidalSearchStructure::FindTrapezoid(const LineSegment2D& segment) const {
        const auto [leftEndPoint, rightEndPoint] = GetEndPoints(segment);
        std::size_t nodePtr = Root;
        while (!Nodes.at(nodePtr).IsLeaf()) {
            auto& internalNode = std::get<1>(Nodes.at(nodePtr).State);
            std::visit(
                Overloads{
                    [&leftEndPoint, &internalNode, &nodePtr](const XNode& xNode) {
                        nodePtr = LexicographicalLess<Vertex2D>::Compare(leftEndPoint, xNode.Vertex)
                                      ? internalNode.LeftChild
                                      : internalNode.RightChild;
                    },
                    [&leftEndPoint, &rightEndPoint, &internalNode, &nodePtr](const YNode& yNode) {
                        const auto [v0, v1] = GetEndPoints(yNode.Segment);
                        const auto side = FindSide(v0, v1, leftEndPoint);
                        if (side == PointSide::Negative) {
                            nodePtr = internalNode.LeftChild;
                        } else if (side == PointSide::Positive) {
                            nodePtr = internalNode.RightChild;
                        } else if (side == PointSide::Collinear) {
                            COMPG_ASSERT(
                                AreEqual(v0, leftEndPoint),
                                "Expected the line segments to intersect at their left endpoints"
                            );
                            const auto angle = Angle180(rightEndPoint - leftEndPoint, v1 - v0);
                            COMPG_ASSERT(!math::IsZero(angle), "The line segments are collinear");
                            nodePtr = (angle < 0) ? internalNode.LeftChild : internalNode.RightChild;

                        } else {
                            COMPG_THROW("Received unexpected value");
                        }
                    }
                },
                internalNode.Node
            );
        }

        return std::get<0>(Nodes.at(nodePtr).State).TrapezoidIndex;
    }

    std::vector<TrapezoidalSearchStructure::trapezoid_index> TrapezoidalSearchStructure::FollowSegment(
        const TrapezoidalMap& trapezoidalMap, const LineSegment2D& segment
    ) const {
        std::vector<trapezoid_index> result{};
        result.push_back(FindTrapezoid(segment));
        const auto [leftEndPoint, rightEndPoint] = GetEndPoints(segment);
        auto trapezoid = trapezoidalMap.GetTrapezoid(result.back());

        while (LexicographicalLess<Vertex2D>::Compare(trapezoid.RightPoint, rightEndPoint)) {
            const auto side = FindSide(leftEndPoint, rightEndPoint, trapezoid.RightPoint);
            if (side == PointSide::Negative) {
                COMPG_ASSERT(trapezoid.LowerRight.has_value(), "Expected the trapezoid to have a lower right neighbor");
                result.push_back(trapezoid.LowerRight.value());
            } else if (side == PointSide::Positive) {
                COMPG_ASSERT(
                    trapezoid.UpperRight.has_value(), "Expected the trapezoid to have an upper right neighbor"
                );
                result.push_back(trapezoid.UpperRight.value());
            } else {
                COMPG_THROW("This should not happen");
            }

            trapezoid = trapezoidalMap.GetTrapezoid(result.back());
        }
        return result;
    }

    void TrapezoidalSearchStructure::UpdateAfterSplit(
        trapezoid_index trapezoidIndex, const TrapezoidalMap::SplitResult& splitResult, const LineSegment2D& segment
    ) {
        const auto [leftEndPoint, rightEndPoint] = GetEndPoints(segment);
        const auto leafNode = TrapezoidLeafNodes.at(trapezoidIndex);

        auto createNewLeaf = [this](trapezoid_index trapezoidIndex) {
            TrapezoidLeafNodes[trapezoidIndex] = Nodes.size();
            Nodes.emplace_back(TrapezoidalSearchLeafNode{trapezoidIndex});
        };
        const auto [topTrapezoidIndex, bottomTrapezoidIndex] = splitResult.TrapezoidIndices.at(0);
        createNewLeaf(topTrapezoidIndex);
        createNewLeaf(bottomTrapezoidIndex);
        TrapezoidalSearchNode node{TrapezoidalSearchInternalNode{
            YNode{segment}, TrapezoidLeafNodes.at(topTrapezoidIndex), TrapezoidLeafNodes.at(bottomTrapezoidIndex)
        }};

        if (splitResult.RightTrapezoidIndex.has_value()) {
            createNewLeaf(splitResult.RightTrapezoidIndex.value());
            Nodes.push_back(node);
            node = {TrapezoidalSearchInternalNode{
                XNode{rightEndPoint}, Nodes.size() - 1, TrapezoidLeafNodes.at(splitResult.RightTrapezoidIndex.value())
            }};
        }

        if (splitResult.LeftTrapezoidIndex.has_value()) {
            createNewLeaf(splitResult.LeftTrapezoidIndex.value());
            Nodes.push_back(node);
            node = {TrapezoidalSearchInternalNode{
                XNode{leftEndPoint}, TrapezoidLeafNodes.at(splitResult.LeftTrapezoidIndex.value()), Nodes.size() - 1
            }};
        }
        Nodes.at(leafNode) = node;
    }

    void TrapezoidalSearchStructure::UpdateAfterSplit(
        const std::vector<trapezoid_index>& trapezoidIndices, const TrapezoidalMap::SplitResult& splitResult,
        const LineSegment2D& segment
    ) {
        if (trapezoidIndices.size() == 1) {
            return UpdateAfterSplit(trapezoidIndices.at(0), splitResult, segment);
        }

        const auto [leftEndPoint, rightEndPoint] = GetEndPoints(segment);
        const auto trapezoidNodePtrs = trapezoidIndices
                                       | std::views::transform([this](auto i) { return TrapezoidLeafNodes.at(i); })
                                       | std::ranges::to<std::vector>();

        auto createNewLeaf = [this](trapezoid_index trapezoidIndex) {
            TrapezoidLeafNodes[trapezoidIndex] = Nodes.size();
            Nodes.emplace_back(TrapezoidalSearchLeafNode{trapezoidIndex});
        };
        if (splitResult.LeftTrapezoidIndex.has_value()) {
            createNewLeaf(splitResult.LeftTrapezoidIndex.value());
        }
        if (splitResult.RightTrapezoidIndex.has_value()) {
            createNewLeaf(splitResult.RightTrapezoidIndex.value());
        }
        std::ranges::for_each(splitResult.TrapezoidIndices, [&createNewLeaf](const auto& tup) {
            const auto [topTrapezoidIndex, bottomTrapezoidIndex] = tup;
            createNewLeaf(topTrapezoidIndex);
            createNewLeaf(bottomTrapezoidIndex);
        });

        TrapezoidalSearchNode leftYNode{TrapezoidalSearchInternalNode{
            YNode{segment}, TrapezoidLeafNodes.at(std::get<0>(splitResult.TrapezoidIndices.at(0))),
            TrapezoidLeafNodes.at(std::get<1>(splitResult.TrapezoidIndices.at(0)))
        }};
        const auto leftNodePtr = trapezoidNodePtrs.at(0);
        if (splitResult.LeftTrapezoidIndex.has_value()) {
            const auto leftTrapezoidIndex = splitResult.LeftTrapezoidIndex.value();
            TrapezoidalSearchNode leftXNode{TrapezoidalSearchInternalNode{
                XNode{leftEndPoint}, TrapezoidLeafNodes.at(leftTrapezoidIndex), Nodes.size()
            }};

            Nodes.at(leftNodePtr) = leftXNode;
            Nodes.push_back(leftYNode);
        } else {
            Nodes.at(leftNodePtr) = leftYNode;
        }

        TrapezoidalSearchNode rightYNode{TrapezoidalSearchInternalNode{
            YNode{segment}, TrapezoidLeafNodes.at(std::get<0>(splitResult.TrapezoidIndices.back())),
            TrapezoidLeafNodes.at(std::get<1>(splitResult.TrapezoidIndices.back()))
        }};
        const auto rightNodePtr = trapezoidNodePtrs.back();
        if (splitResult.RightTrapezoidIndex.has_value()) {
            const auto rightTrapezoidIndex = splitResult.RightTrapezoidIndex.value();
            TrapezoidalSearchNode rightXNode{TrapezoidalSearchInternalNode{
                XNode{rightEndPoint}, Nodes.size(), TrapezoidLeafNodes.at(rightTrapezoidIndex)
            }};

            Nodes.at(rightNodePtr) = rightXNode;
            Nodes.push_back(rightYNode);
        } else {
            Nodes.at(rightNodePtr) = rightYNode;
        }

        for (const auto [i, trapezoidIndex] : trapezoidIndices | std::views::enumerate
                                                  | std::views::take(trapezoidIndices.size() - 1)
                                                  | std::views::drop(1)) {
            const TrapezoidalSearchNode node{TrapezoidalSearchInternalNode{
                YNode{segment}, TrapezoidLeafNodes.at(std::get<0>(splitResult.TrapezoidIndices.at(i))),
                TrapezoidLeafNodes.at(std::get<1>(splitResult.TrapezoidIndices.at(i)))
            }};
            Nodes.at(trapezoidNodePtrs.at(i)) = node;
        }
    }
} // namespace compg