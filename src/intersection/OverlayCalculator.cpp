#include "intersection/OverlayCalculator.hpp"

#include "data_structures/BinarySearchTree.hpp"
#include "data_structures/DisjointSetUnion.hpp"
#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include "intersection/IntersectionCalculatorState.hpp"
#include "range_query/layered_range_tree/LayeredRangeTree.hpp"
#include "range_query/layered_range_tree/LayeredRangeTreeQuery.hpp"

namespace compg {
    namespace details {
        bool IsIntersectionBetweenEdgeLists(const auto& indices, const OverlayCalculatorState& state) {
            auto hasEdge = [&indices, &state](const auto& edgeMap) {
                return std::ranges::any_of(
                           indices.LowerIndices,
                           [&edgeMap, &state](auto i) { return edgeMap.contains(state.Map.at(i)); }
                       )
                       || std::ranges::any_of(
                           indices.CenterIndices,
                           [&edgeMap, &state](auto i) { return edgeMap.contains(state.Map.at(i)); }
                       )
                       || std::ranges::any_of(indices.UpperIndices, [&edgeMap, &state](auto i) {
                              return edgeMap.contains(state.Map.at(i));
                          });
            };
            return hasEdge(state.LeftEdges) && hasEdge(state.RightEdges);
        }

        DisjointSetUnion FindDisjointSets(const std::vector<Vertex2D>& vertices, Float epsilon) {
            DisjointSetUnion setUnion{vertices.size()};
            const LayeredRangeTree<2> tree{vertices};
            LayeredRangeTreeQuery<2> query;
            for (const auto& [i, v] : vertices | std::views::enumerate) {
                const Vertex2D e{epsilon, epsilon};
                const LayeredRangeTreeQueryRegion<2> range{Box2D{v - e, v + e}};
                const auto queryResult = query.Query(tree, range);
                for (const auto& [j, w] : queryResult) {
                    setUnion.Union(i, j);
                }
            }

            return setUnion;
        }

        void SplitEdges(
            const auto& edgeMap, DisjointSetUnion& setUnion, std::size_t splitIndex, DoublyConnectedEdgeList& edgeList1,
            DoublyConnectedEdgeList& edgeList2, auto comparator, auto projector
        ) {
            for (const auto& [root, equivalentIndices] : setUnion.FindDisjointSets()) {
                AvlTree<Vertex2D> tree;
                for (const auto i : equivalentIndices) {
                    const auto segment = i < splitIndex ? edgeList1.GetEdgeAsLineSegment(edgeMap.at(i))
                                                        : edgeList2.GetEdgeAsLineSegment(edgeMap.at(i));
                    tree.Insert(segment[0], comparator, projector);
                    tree.Insert(segment[1], comparator, projector);
                }

                for (const auto i : equivalentIndices) {
                    auto edgeIndex = edgeMap.at(i);
                    auto& edgeList = i < splitIndex ? edgeList1 : edgeList2;
                    const auto segment = edgeList.GetEdgeAsLineSegment(edgeIndex);

                    if (comparator(projector(segment[0]), projector(segment[1]))) {
                        edgeIndex = edgeList.GetTwinIndex(edgeIndex);
                    }

                    const auto [v0, v1] = comparator(projector(segment[0]), projector(segment[1]))
                                              ? std::tuple{segment[0], segment[1]}
                                              : std::tuple{segment[1], segment[0]};
                    auto it1 = tree.UpperBound(projector(v0), comparator, projector);
                    COMPG_ASSERT(it1 != tree.end(), "Expected a valid iterator");
                    const auto it2 = tree.Find(projector(v1), comparator, projector);
                    COMPG_ASSERT(it2 != tree.end(), "Expected a valid iterator");

                    while (it1 != it2) {
                        edgeList.Split(edgeIndex, *it1);
                        ++it1;
                    }
                }
            }
        }

        auto SplitOverlappingVerticalEdges(
            DoublyConnectedEdgeList& edgeList1, DoublyConnectedEdgeList& edgeList2, Float epsilon = EPSILON
        ) {
            std::vector<Vertex2D> intercepts;
            std::unordered_map<std::size_t, DoublyConnectedEdgeList::edge_index> edgeMap;

            auto visitEdge = [&intercepts, &edgeMap](const auto& edgeList) {
                return [&edgeList, &intercepts, &edgeMap, vertical = MakeLine2DFromX(0.0)](auto edgeIndex) {
                    const auto segment = edgeList.GetEdgeAsLineSegment(edgeIndex);
                    const Line2D line{segment[0], segment[1]};
                    if (const auto intersection = FindIntersection(vertical, line); !intersection.has_value()) {
                        edgeMap[intercepts.size()] = edgeIndex;
                        intercepts.emplace_back(segment[0][0], 0);
                    }
                };
            };

            WalkUndirectedEdges(edgeList1, visitEdge(edgeList1));
            const auto splitIndex = intercepts.size();
            WalkUndirectedEdges(edgeList2, visitEdge(edgeList2));
            if (intercepts.empty()) {
                return;
            }
            auto setUnion = FindDisjointSets(intercepts, epsilon);
            SplitEdges(edgeMap, setUnion, splitIndex, edgeList1, edgeList2, Less{}, CoordinateProjector<2>{1});
        }

        auto SplitOverlappingNonVerticalEdges(
            DoublyConnectedEdgeList& edgeList1, DoublyConnectedEdgeList& edgeList2, Float epsilon = EPSILON
        ) {
            std::vector<Vertex2D> slopeIntercepts;
            slopeIntercepts.reserve(edgeList1.NumEdges() / 2 + edgeList2.NumEdges() / 2);

            std::unordered_map<std::size_t, DoublyConnectedEdgeList::edge_index> edgeMap;
            auto visitEdge = [&slopeIntercepts, &edgeMap](const auto& edgeList) {
                return [&edgeList, &slopeIntercepts, &edgeMap](auto edgeIndex) {
                    const auto segment = edgeList.GetEdgeAsLineSegment(edgeIndex);
                    const Line2D line{segment[0], segment[1]};
                    const auto vertical = MakeLine2DFromX(0.0);
                    if (const auto intersection = FindIntersection(vertical, line)) {
                        const auto slope = (segment[0][1] - segment[1][1]) / (segment[0][0] - segment[1][0]);
                        edgeMap[slopeIntercepts.size()] = edgeIndex;
                        slopeIntercepts.emplace_back(slope, intersection.value()[1]);
                    }
                };
            };
            WalkUndirectedEdges(edgeList1, visitEdge(edgeList1));
            const auto splitIndex = slopeIntercepts.size();
            WalkUndirectedEdges(edgeList2, visitEdge(edgeList2));
            if (slopeIntercepts.empty()) {
                return;
            }
            auto setUnion = FindDisjointSets(slopeIntercepts, epsilon);
            SplitEdges(
                edgeMap, setUnion, splitIndex, edgeList1, edgeList2, LexicographicalLess<Vertex2D>{}, Identity{}
            );
        }

        auto SplitOverlappingEdges(
            DoublyConnectedEdgeList& edgeList1, DoublyConnectedEdgeList& edgeList2, Float epsilon = EPSILON
        ) {
            SplitOverlappingNonVerticalEdges(edgeList1, edgeList2, epsilon);
            SplitOverlappingVerticalEdges(edgeList1, edgeList2, epsilon);
        }
    } // namespace details

    DoublyConnectedEdgeList
    OverlayCalculator::FindOverlay(const DoublyConnectedEdgeList& lhs, const DoublyConnectedEdgeList& rhs) const {
        auto lhs_{lhs};
        auto rhs_{rhs};
        details::SplitOverlappingEdges(lhs_, rhs_);

        auto [overlay, leftEdges, rightEdges] = UnsafeUnion(lhs_, rhs_);
        std::vector<LineSegment2D> segments;
        segments.reserve(overlay.NumEdges() / 2);
        auto map = InsertEdges(overlay, segments);

        details::OverlayCalculatorState overlayState{std::move(map), std::move(leftEdges), std::move(rightEdges)};

        std::unordered_map<Vertex2D, DoublyConnectedEdgeList::edge_index> leftNeighborEdgeMap;

        IntersectionCalculatorState intersectionState{segments, Epsilon};
        auto callback = [&leftNeighborEdgeMap, &overlayState, &overlay](
                            const Vertex2D& event, auto maybeLeftIt, auto, const EventPointSegmentIndices& indices
                        ) {
            if (maybeLeftIt) {
                const auto segmentIndex = **maybeLeftIt;
                const auto edgeIndex = overlayState.Map.at(segmentIndex);
                leftNeighborEdgeMap[event] = overlay.GetTwinIndex(edgeIndex);
            }

            if (details::IsIntersectionBetweenEdgeLists(indices, overlayState)) {
                if (indices.CenterIndices.empty()) {
                    HandlePointPointIntersection(
                        event, indices.LowerIndices, indices.UpperIndices, overlayState, overlay
                    );
                } else if (indices.CenterIndices.size() == 1) {
                    HandlePointLineIntersection(
                        event, indices.LowerIndices, indices.CenterIndices.at(0), indices.UpperIndices, overlayState,
                        overlay
                    );
                } else if (indices.CenterIndices.size() == 2) {
                    COMPG_ASSERT(indices.LowerIndices.empty(), "This should be impossible in a valid edge list");
                    COMPG_ASSERT(indices.UpperIndices.empty(), "This should be impossible in a valid edge list");
                    HandleLineLineIntersection(
                        event, indices.CenterIndices.at(0), indices.CenterIndices.at(1), overlayState, overlay
                    );
                } else {
                    COMPG_THROW(
                        "More than two segments intersect each other in the center. This should be impossible in valid "
                        "edge lists"
                    );
                }
            }
        };

        intersectionState.Run(callback);
        UpdateFaces(leftNeighborEdgeMap, overlay);

        return overlay;
    }
} // namespace compg