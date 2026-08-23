#include "discrepancy/ArrangementCalculator.hpp"

#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include "math/Geometry.hpp"
#include "math/primitives/Box.hpp"

namespace compg {
    namespace details {
        auto FindBoundingBox(const std::vector<Line2D>& lines) {
            Float xMin = Infinity;
            Float xMax = -Infinity;
            Float yMin = Infinity;
            Float yMax = -Infinity;
            bool foundIntersection = false;

            std::ranges::for_each(std::views::cartesian_product(lines, lines), [&](const auto& tup) {
                const auto& [l1, l2] = tup;
                if (const auto intersection = FindIntersection(l1, l2); intersection.has_value()) {
                    foundIntersection = true;
                    xMin = std::min(xMin, intersection.value()[0]);
                    xMax = std::max(xMax, intersection.value()[0]);
                    yMin = std::min(yMin, intersection.value()[1]);
                    yMax = std::max(yMax, intersection.value()[1]);
                };
            });

            return foundIntersection ? std::optional<Box2D>{{{xMin - 1, yMin - 1}, {xMax + 1, yMax + 1}}}
                                     : std::nullopt;
        }

        auto CreateInitialArrangement(const Box2D& boundingBox) {
            const Vertex2D& v0 = boundingBox.GetLowerCorner();
            const Vertex2D& v2 = boundingBox.GetUpperCorner();
            const Vertex2D v1{v2.x(), v0.y()};
            const Vertex2D v3{v0.x(), v2.y()};

            DoublyConnectedEdgeList arrangement;
            const auto i0 = arrangement.InsertVertex(v0);
            const auto i1 = arrangement.InsertVertex(v1);
            const auto i2 = arrangement.InsertVertex(v2);
            const auto i3 = arrangement.InsertVertex(v3);
            arrangement.InsertEdge(i0, i1);
            arrangement.InsertEdge(i2, i1);
            arrangement.InsertEdge(i2, i3);
            arrangement.InsertEdge(i0, i3);
            UpdateFaces(arrangement);

            return arrangement;
        }

        auto FindLeftMostIntersection(
            const DoublyConnectedEdgeList& edgeList, const Line2D& line,
            DoublyConnectedEdgeList::edge_index boundaryEdge
        ) {
            std::optional<std::tuple<DoublyConnectedEdgeList::edge_index, Vertex2D>> leftMost;
            WalkBoundary(
                edgeList, boundaryEdge, [&line, &edgeList, &leftMost](DoublyConnectedEdgeList::edge_index edgeIndex) {
                    const auto segment = edgeList.GetEdgeAsLineSegment(edgeIndex);
                    if (const auto intersection = FindIntersection(segment, line); intersection) {
                        if (!leftMost.has_value()
                            || LexicographicalLess<Vertex2D>::Compare(
                                intersection.value(), std::get<1>(leftMost.value())
                            )) {
                            leftMost = std::tuple{edgeIndex, intersection.value()};
                        }
                    }
                }
            );

            COMPG_ASSERT(leftMost.has_value(), "Expected at least one edge to intersect the line");
            return leftMost.value();
        }

        auto HandleVertexInsertion(
            DoublyConnectedEdgeList::edge_index edgeIndex, const Vertex2D& intersection, const Line2D& line,
            DoublyConnectedEdgeList& edgeList
        ) {
            if (edgeList.HasVertex(intersection)) {
                const auto [vMin, vMax] = LexicographicalLess<Vertex2D>::Compare(line[0], line[1])
                                              ? std::tuple{line[0], line[1]}
                                              : std::tuple{line[1], line[0]};
                const auto direction = vMax - vMin;
                const auto intersectionIndex = edgeList.GetVertexIndex(intersection);
                const auto nextEdge = FindCyclicNext(edgeList, Vertex2D{intersection + direction}, intersectionIndex);
                COMPG_ASSERT(nextEdge.has_value(), "Expected a valid edge index");
                return std::tuple{nextEdge.value(), intersectionIndex};
            }
            const auto intersectionIndex = edgeList.Split(edgeIndex, intersection);
            return std::tuple{edgeList.GetTwinIndex(edgeIndex), intersectionIndex};
        }

        auto FindFirstIntersection(
            const DoublyConnectedEdgeList& edgeList, const Line2D& line,
            DoublyConnectedEdgeList::edge_index boundaryEdge
        ) {
            auto edgeIndex = boundaryEdge;
            do {
                const auto edge = edgeList.GetEdgeAsLineSegment(edgeIndex);
                const auto intersection = FindIntersection(line, edge);
                if (intersection.has_value()) {
                    return std::tuple{edgeIndex, intersection.value()};
                }
                edgeIndex = edgeList.GetNextIndex(edgeIndex);
            } while (edgeIndex != boundaryEdge);

            COMPG_THROW("Expected the line to intersect the boundary");
        }

        auto InsertLine(const Line2D& line, DoublyConnectedEdgeList& arrangement) {
            const auto unboundedFace = arrangement.FindUnboundedFaceIndex();
            const auto outerEdge = arrangement.GetFace(unboundedFace).InnerComponents.at(0);
            const auto [startEdge, intersection] = details::FindLeftMostIntersection(arrangement, line, outerEdge);
            auto [boundaryEdge, leftIndex] = details::HandleVertexInsertion(startEdge, intersection, line, arrangement);
            std::vector<DoublyConnectedEdgeList::edge_index> leftToRightEdges;

            while (arrangement.GetFaceIndex(boundaryEdge) != unboundedFace) {
                const auto [rightEdge, rightVertex]
                    = details::FindFirstIntersection(arrangement, line, arrangement.GetNextIndex(boundaryEdge));
                const auto [nextBoundaryEdge, rightIndex]
                    = details::HandleVertexInsertion(rightEdge, rightVertex, line, arrangement);
                COMPG_ASSERT(leftIndex != rightIndex, "Found the same intersection point");
                const auto newEdgeIndex = arrangement.SplitFace(leftIndex, rightIndex);
                leftToRightEdges.push_back(newEdgeIndex);

                boundaryEdge = nextBoundaryEdge;
                leftIndex = rightIndex;
            }

            return leftToRightEdges;
        }

        auto RemapEdgeMap(
            const ArrangementCalculator::ArrangementDetails::map_type& edgeMap, const auto& vertexMap,
            const DoublyConnectedEdgeList& edgeList, const DoublyConnectedEdgeList& edgeListCollapsed
        ) {
            ArrangementCalculator::ArrangementDetails::map_type remapped;
            std::ranges::copy(
                edgeMap | std::views::transform([&vertexMap, &edgeList, &edgeListCollapsed](const auto& pair) {
                    const auto& [lineIndex, edgeIndex] = pair;
                    const auto v1 = vertexMap.at(edgeList.GetOriginIndex(edgeIndex));
                    const auto v2 = vertexMap.at(edgeList.GetDestinationIndex(edgeIndex));
                    return std::pair{lineIndex, edgeListCollapsed.GetEdgeIndex(v1, v2)};
                }),
                std::inserter(remapped, remapped.end())
            );

            return remapped;
        }
    } // namespace details

    DoublyConnectedEdgeList ArrangementCalculator::FindArrangement(const std::vector<Line2D>& lines) const {
        const auto maybeBoundingBox = details::FindBoundingBox(lines);
        COMPG_ASSERT(maybeBoundingBox.has_value(), "Expected the lines to have at least one intersection");
        const auto& boundingBox = maybeBoundingBox.value();

        auto arrangement = details::CreateInitialArrangement(boundingBox);
        std::ranges::for_each(lines, [&arrangement](const auto& line) { details::InsertLine(line, arrangement); });

        auto [arrangementCollapsed, vertexMap] = CollapseVertices(arrangement);
        UpdateFaces(arrangementCollapsed);
        return arrangementCollapsed;
    }

    ArrangementCalculator::ArrangementDetails
    ArrangementCalculator::FindArrangementDetails(const std::vector<Line2D>& lines) const {
        const auto maybeBoundingBox = details::FindBoundingBox(lines);
        COMPG_ASSERT(maybeBoundingBox.has_value(), "Expected the lines to have at least one intersection");
        const auto& boundingBox = maybeBoundingBox.value();

        ArrangementDetails::map_type firstEdgeMap;

        auto arrangement = details::CreateInitialArrangement(boundingBox);
        std::ranges::for_each(lines | std::views::enumerate, [&arrangement, &firstEdgeMap](const auto& tup) {
            const auto& [index, line] = tup;
            const auto halfEdges = details::InsertLine(line, arrangement);

            firstEdgeMap[index] = halfEdges.at(0);
        });

        auto [arrangementCollapsed, vertexMap] = CollapseVertices(arrangement);
        UpdateFaces(arrangementCollapsed);
        const auto firstEdgeMapCollapsed
            = details::RemapEdgeMap(firstEdgeMap, vertexMap, arrangement, arrangementCollapsed);

        return ArrangementDetails{.Arrangement = arrangementCollapsed, .FirstEdgeMap = firstEdgeMapCollapsed};
    }
} // namespace compg
