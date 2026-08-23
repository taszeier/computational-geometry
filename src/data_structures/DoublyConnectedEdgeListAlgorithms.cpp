#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include "data_structures/DisjointSetUnion.hpp"
#include "data_structures/VertexEquivalenceCalculator.hpp"
#include "data_structures/VertexFinder.hpp"
#include "intersection/IntersectionCalculatorState.hpp"
#include "math/Geometry.hpp"
#include "range_query/kd_tree/BallQueryRegion.hpp"
#include "sweep_line/SweepLine.hpp"

namespace compg {
    using edge_index = DoublyConnectedEdgeList::edge_index;
    using vertex_index = DoublyConnectedEdgeList::vertex_index;
    using edge_type = DoublyConnectedEdgeList::edge_type;

    std::vector<edge_index> FindBoundaries(const DoublyConnectedEdgeList& edgeList) {
        std::unordered_set<edge_index> visited;
        std::vector<edge_index> boundaries;

        std::ranges::for_each(
            std::views::iota(0UZ, edgeList.NumEdges()), [&edgeList, &visited, &boundaries](edge_index edgeIndex) {
                if (!visited.contains(edgeIndex)) {
                    boundaries.push_back(edgeIndex);
                    auto visitEdge = [&visited](edge_index e) { visited.insert(e); };
                    WalkBoundary(edgeList, edgeIndex, visitEdge);
                }
            }
        );

        return boundaries;
    }

    std::vector<std::unordered_set<edge_index>> FindAllBoundaryEdges(const DoublyConnectedEdgeList& edgeList) {
        std::unordered_set<edge_index> visited;
        std::vector<std::unordered_set<edge_index>> boundaries;

        std::ranges::for_each(
            std::views::iota(0UZ, edgeList.NumEdges()), [&edgeList, &visited, &boundaries](edge_index edgeIndex) {
                if (!visited.contains(edgeIndex)) {
                    boundaries.emplace_back();
                    auto visitEdge = [&boundaries, &visited](edge_index e) {
                        visited.insert(e);
                        boundaries.back().insert(e);
                    };
                    WalkBoundary(edgeList, edgeIndex, visitEdge);
                }
            }
        );

        return boundaries;
    }

    std::vector<DoublyConnectedEdgeList::vertex_index>
    FindBoundaryVertices(const DoublyConnectedEdgeList& edgeList, DoublyConnectedEdgeList::edge_index edgeIndex) {
        std::vector<DoublyConnectedEdgeList::vertex_index> vertices;
        WalkBoundary(edgeList, edgeIndex, [&edgeList, &vertices](auto edgeIndex) {
            vertices.push_back(edgeList.GetOriginIndex(edgeIndex));
        });
        return vertices;
    }

    edge_index FindLeftMostEdge(const DoublyConnectedEdgeList& edgeList, edge_index edgeIndex) {
        auto leftMostVertexEdge = edgeIndex;

        WalkBoundary(
            edgeList, edgeIndex,
            [&leftMostVertexEdge, &edgeList, comparator = LexicographicalLess<Vertex2D>{}](edge_index e) {
                const auto currentVertexIndex = edgeList.GetOriginIndex(e);
                const auto leftMostVertexIndex = edgeList.GetOriginIndex(leftMostVertexEdge);

                if (comparator(
                        edgeList.GetVertex(currentVertexIndex).Vertex, edgeList.GetVertex(leftMostVertexIndex).Vertex
                    )) {

                    leftMostVertexEdge = e;
                }
            }
        );

        return leftMostVertexEdge;
    }

    bool IsOuterBoundary(const DoublyConnectedEdgeList& edgeList, edge_index edgeIndex) {
        const auto leftMostVertexEdge = FindLeftMostEdge(edgeList, edgeIndex);

        const auto previousEdgeIndex = edgeList.GetTwinIndex(edgeList.GetPreviousIndex(leftMostVertexEdge));
        const auto v2 = edgeList.GetEdgeAsVector(previousEdgeIndex);
        const auto v1 = edgeList.GetEdgeAsVector(leftMostVertexEdge);
        const auto interiorAngle = Angle360(v1, v2);

        return 0 < interiorAngle && interiorAngle < math::PI;
    }

    std::vector<edge_index> FindOutgoingEdges(const DoublyConnectedEdgeList& edgeList, vertex_index vertexIndex) {
        std::vector<edge_index> edges;
        WalkOutgoingEdges(edgeList, vertexIndex, [&edges](edge_index e) { edges.push_back(e); });

        return edges;
    }

    std::optional<edge_index>
    FindCyclicNext(const DoublyConnectedEdgeList& edgeList, const Vertex2D& origin, vertex_index destination) {
        const Vertex2D reversed = origin - edgeList.GetVertex(destination).Vertex;
        auto angleBetween = [&edgeList, &reversed](edge_index edgeIndex) {
            const auto outgoingEdge = edgeList.GetEdgeAsVector(edgeIndex);
            return Angle360(outgoingEdge, reversed);
        };

        const auto incidentEdge = edgeList.GetVertex(destination).IncidentEdgeIndex;
        if (!incidentEdge.has_value()) {
            return std::nullopt;
        }

        edge_index result = incidentEdge.value();
        WalkOutgoingEdges(edgeList, destination, [&result, &angleBetween](edge_index edgeIndex) {
            if (angleBetween(edgeIndex) < angleBetween(result)) {
                result = edgeIndex;
            }
        });
        COMPG_ASSERT(
            !math::IsZero(angleBetween(result)), "The vertex cannot be inserted without creating an invalid edge list"
        );
        return result;
    }

    UnsafeUnionResult UnsafeUnion(const DoublyConnectedEdgeList& lhs, const DoublyConnectedEdgeList& rhs) {
        auto result{lhs};
        result.AreFacesValid_ = false;
        const auto leftEdges = std::views::iota(0UZ, lhs.NumEdges())
                               | std::ranges::to<std::unordered_set<DoublyConnectedEdgeList::edge_index>>();

        std::ranges::for_each(rhs.Vertices, [&result](const auto& vertexRecord) {
            result.InsertVertex(vertexRecord.Vertex);
        });

        std::unordered_map<edge_index, edge_index> rhsEdgeMap;

        std::ranges::for_each(rhs.Edges | std::views::enumerate, [&result, &rhs, &rhsEdgeMap](const auto& tup) {
            const auto& [index, edgeRecord] = tup;
            const auto originIndex = result.VertexIndexMap.at(rhs.GetVertex(edgeRecord.OriginIndex).Vertex);
            const auto destinationIndex
                = result.VertexIndexMap.at(rhs.GetVertex(rhs.GetOriginIndex(edgeRecord.TwinIndex)).Vertex);
            const DoublyConnectedEdgeList::edge_type edge{originIndex, destinationIndex};

            if (!result.EdgeIndexMap.contains(edge)) {
                const auto unionEdgeIndex = result.Edges.size();
                result.EdgeIndexMap[edge] = unionEdgeIndex;
                if (!result.Vertices.at(originIndex).IncidentEdgeIndex.has_value()) {
                    result.Vertices.at(originIndex).IncidentEdgeIndex = unionEdgeIndex;
                }
                result.Edges.emplace_back(originIndex, 0, 0, 0, 0);
            }
            rhsEdgeMap[index] = result.EdgeIndexMap.at(edge);
        });

        for (const auto [rhsEdgeIndex, resultEdgeIndex] : rhsEdgeMap) {
            if (resultEdgeIndex >= lhs.NumEdges()) {
                result.Edges.at(resultEdgeIndex).TwinIndex = rhsEdgeMap.at(rhs.GetTwinIndex(rhsEdgeIndex));
                result.Edges.at(resultEdgeIndex).NextIndex = rhsEdgeMap.at(rhs.GetNextIndex(rhsEdgeIndex));
                result.Edges.at(resultEdgeIndex).PreviousIndex = rhsEdgeMap.at(rhs.GetPreviousIndex(rhsEdgeIndex));
            }
        }

        const auto rightEdges = rhsEdgeMap | std::views::values | std::ranges::to<std::unordered_set>();
        return {result, leftEdges, rightEdges};
    }

    namespace details {
        auto CreateBoundaryMap(const DoublyConnectedEdgeList& edgeList, const std::vector<edge_index>& boundaries) {
            std::unordered_map<DoublyConnectedEdgeList::edge_index, std::size_t> boundaryMap;
            std::ranges::for_each(boundaries | std::views::enumerate, [&boundaryMap, &edgeList](const auto pair) {
                const auto [boundaryIndex, edgeIndex] = pair;
                WalkBoundary(edgeList, edgeIndex, [&boundaryMap, boundaryIndex](auto e) {
                    boundaryMap[e] = boundaryIndex;
                });
            });

            return boundaryMap;
        }

        auto FindOuterBoundaries(const DoublyConnectedEdgeList& edgeList, const std::vector<edge_index>& boundaries) {
            std::unordered_set<std::size_t> outerBoundaries;
            outerBoundaries.insert(boundaries.size());
            std::ranges::for_each(boundaries | std::views::enumerate, [&edgeList, &outerBoundaries](const auto pair) {
                const auto [boundaryIndex, edgeIndex] = pair;
                if (IsOuterBoundary(edgeList, edgeIndex)) {
                    outerBoundaries.insert(boundaryIndex);
                }
            });

            return outerBoundaries;
        }

        auto FindDisjointBoundarySetUnion(
            const DoublyConnectedEdgeList& edgeList, const auto& leftNeighborEdgeMap,
            const std::vector<edge_index>& boundaries, const auto& boundaryMap, const auto& outerBoundaries
        ) {
            DisjointSetUnion setUnion{boundaries.size() + 1};

            std::ranges::for_each(
                boundaries | std::views::enumerate,
                [&edgeList, &leftNeighborEdgeMap, &boundaryMap, &setUnion, &outerBoundaries,
                 unboundedFaceBoundaryIndex = boundaries.size()](const auto pair) {
                    const auto [boundaryIndex, edgeIndex] = pair;
                    if (!outerBoundaries.contains(boundaryIndex)) {
                        const auto leftMostEdgeIndex = FindLeftMostEdge(edgeList, edgeIndex);
                        const auto leftMostVertex
                            = edgeList.GetVertex(edgeList.GetOriginIndex(leftMostEdgeIndex)).Vertex;
                        if (leftNeighborEdgeMap.contains(leftMostVertex)) {
                            const auto leftBoundaryEdgeIndex = leftNeighborEdgeMap.at(leftMostVertex);
                            const auto leftBoundaryIndex = boundaryMap.at(leftBoundaryEdgeIndex);

                            setUnion.Union(boundaryIndex, leftBoundaryIndex);
                        } else {
                            setUnion.Union(boundaryIndex, unboundedFaceBoundaryIndex);
                        }
                    }
                }
            );

            return setUnion;
        }

        auto FindOuterBoundaryIndex(const auto& boundaryIndices, const auto& outerBoundaries) {
            auto isOuterBoundary
                = [&outerBoundaries](auto boundaryIndex) { return outerBoundaries.contains(boundaryIndex); };
            const auto outerBoundaryIt = std::ranges::find_if(boundaryIndices, isOuterBoundary);
            COMPG_ASSERT(outerBoundaryIt != boundaryIndices.end(), "Could not find an outer boundary");
            const auto it2 = std::find_if(std::next(outerBoundaryIt), boundaryIndices.end(), isOuterBoundary);
            COMPG_ASSERT(it2 == boundaryIndices.end(), "Found multiple outer boundaries");

            return *outerBoundaryIt;
        }
    } // namespace details

    void UpdateFaces(
        const std::unordered_map<Vertex2D, DoublyConnectedEdgeList::edge_index>& leftNeighborEdgeMap,
        DoublyConnectedEdgeList& edgeList
    ) {
        const auto boundaries = FindBoundaries(edgeList);
        const auto boundaryMap = details::CreateBoundaryMap(edgeList, boundaries);
        const auto outerBoundaries = details::FindOuterBoundaries(edgeList, boundaries);
        DisjointSetUnion boundarySetUnion = details::FindDisjointBoundarySetUnion(
            edgeList, leftNeighborEdgeMap, boundaries, boundaryMap, outerBoundaries
        );
        const auto boundarySets = boundarySetUnion.FindDisjointSets();

        edgeList.Faces.clear();
        std::ranges::for_each(boundarySets, [&outerBoundaries, &boundaries, &edgeList](const auto& pair) {
            const auto& [rootBoundaryIndex, boundaryIndices] = pair;

            const auto outerBoundaryIndex = details::FindOuterBoundaryIndex(boundaryIndices, outerBoundaries);
            auto outerComponent = outerBoundaryIndex < boundaries.size()
                                      ? std::optional{boundaries.at(outerBoundaryIndex)}
                                      : std::nullopt;
            auto innerComponents
                = boundaryIndices | std::views::filter([outerBoundaryIndex](auto i) { return i != outerBoundaryIndex; })
                  | std::views::transform([&boundaries](auto i) { return boundaries.at(i); })
                  | std::ranges::to<std::vector>();
            edgeList.Faces.emplace_back(outerComponent, std::move(innerComponents));

            std::ranges::for_each(boundaryIndices, [&edgeList, &boundaries](auto boundaryIndex) {
                if (boundaryIndex != boundaries.size()) {
                    WalkBoundary(edgeList, boundaries.at(boundaryIndex), [&edgeList](auto edgeIndex) {
                        edgeList.Edges.at(edgeIndex).FaceIndex = edgeList.Faces.size() - 1;
                    });
                }
            });
        });
        edgeList.AreFacesValid_ = true;
    }

    void UpdateFaces(DoublyConnectedEdgeList& edgeList) {
        std::vector<LineSegment2D> segments;
        segments.reserve(edgeList.NumEdges() / 2);
        const auto edgeMap = InsertEdges(edgeList, segments);

        std::unordered_map<Vertex2D, edge_index> leftNeighborEdgeMap;

        IntersectionCalculatorState intersectionState{segments};
        auto callback = [&leftNeighborEdgeMap, &edgeMap,
                         &edgeList](const Vertex2D& event, auto maybeLeftIt, auto, const EventPointSegmentIndices&) {
            if (maybeLeftIt) {
                const auto segmentIndex = **maybeLeftIt;
                const auto edgeIndex = edgeMap.at(segmentIndex);
                leftNeighborEdgeMap[event] = edgeList.GetTwinIndex(edgeIndex);
            }
        };

        intersectionState.Run(callback);
        UpdateFaces(leftNeighborEdgeMap, edgeList);
    };

    std::unordered_map<std::size_t, DoublyConnectedEdgeList::edge_index>
    InsertEdges(const DoublyConnectedEdgeList& edgeList, std::vector<LineSegment2D>& segments) {
        std::unordered_map<std::size_t, DoublyConnectedEdgeList::edge_index> edgeMap;
        WalkUndirectedEdges(edgeList, [&edgeMap, &edgeList, &segments](auto edgeIndex) {
            if (auto twinIndex = edgeList.GetTwinIndex(edgeIndex); VertexAboveComparator::Compare(
                    edgeList.GetVertex(edgeList.GetOriginIndex(edgeIndex)).Vertex,
                    edgeList.GetVertex(edgeList.GetOriginIndex(twinIndex)).Vertex
                )) {

                edgeIndex = twinIndex;
            }
            edgeMap[segments.size()] = edgeIndex;
            segments.push_back(edgeList.GetEdgeAsLineSegment(edgeIndex));
        });
        return edgeMap;
    }

    std::optional<DoublyConnectedEdgeList::Isomorphism>
    FindIsomorphism(const DoublyConnectedEdgeList& from, const DoublyConnectedEdgeList& to, Float epsilon) {
        if (from.NumVertices() != to.NumVertices() || from.NumEdges() != to.NumEdges()) {
            return std::nullopt;
        }
        if (from.NumVertices() == 0) {
            return DoublyConnectedEdgeList::Isomorphism{};
        }

        const auto vertices = std::views::iota(0UZ, to.NumVertices())
                              | std::views::transform([&to](const auto i) { return to.GetVertex(i).Vertex; })
                              | std::ranges::to<std::vector>();
        VertexFinder<2UZ> finder{vertices};

        std::unordered_map<vertex_index, vertex_index> vertexIsomorphism;
        vertexIsomorphism.reserve(from.NumVertices());
        for (const auto& [vertex, vertexIndex] : from.VertexIndexMap) {
            const auto is = finder.Find(vertex, epsilon);
            if (is.size() != 1) {
                return std::nullopt;
            }
            vertexIsomorphism[vertexIndex] = is.at(0);
        }

        std::unordered_map<edge_index, edge_index> edgeIsomorphism;
        edgeIsomorphism.reserve(from.NumEdges());
        for (const auto& [edge, edgeIndex] : from.EdgeIndexMap) {
            const edge_type edgeMapped{vertexIsomorphism.at(edge.At(0)), vertexIsomorphism.at(edge.At(1))};
            if (!to.EdgeIndexMap.contains(edgeMapped)) {
                return std::nullopt;
            }
            edgeIsomorphism[edgeIndex] = to.EdgeIndexMap.at(edgeMapped);
        }

        return DoublyConnectedEdgeList::Isomorphism{vertexIsomorphism, edgeIsomorphism};
    }

    DoublyConnectedEdgeList Optimize(const DoublyConnectedEdgeList& edgeList, Float epsilon) {
        // TODO: refactor this mess.
        const VertexEquivalenceCalculator equivalence{epsilon};
        DisjointSetUnion vertexSetUnion = equivalence.FindEquivalenceClasses(
            std::views::iota(0UZ, edgeList.NumVertices())
            | std::views::transform([&edgeList](const auto i) { return edgeList.GetVertex(i).Vertex; })
            | std::ranges::to<std::vector>()
        );

        auto getEdgeRep = [&edgeList](const DoublyConnectedEdgeList::edge_index edgeIndex) {
            return std::min(edgeIndex, edgeList.GetTwinIndex(edgeIndex));
        };

        std::unordered_map<DoublyConnectedEdgeList::edge_index, std::size_t> edgeIndexMap;
        WalkUndirectedEdges(edgeList, [&getEdgeRep, &edgeIndexMap](DoublyConnectedEdgeList::edge_index edgeIndex) {
            const DoublyConnectedEdgeList::edge_index rep = getEdgeRep(edgeIndex);
            const std::size_t index = edgeIndexMap.size();
            edgeIndexMap[rep] = index;
        });

        const auto disjointSets = vertexSetUnion.FindDisjointSets();

        std::unordered_set<DoublyConnectedEdgeList::edge_index> visited;
        auto goBackwards = [&](DoublyConnectedEdgeList::edge_index edgeIndex) {
            bool done = false;
            auto ee = edgeIndex;
            do {
                visited.insert(ee);
                visited.insert(edgeList.GetTwinIndex(ee));
                const auto origin = edgeList.GetOriginIndex(ee);
                std::size_t numIncidentEdges = 0;
                const auto equivalentVertices = disjointSets.at(vertexSetUnion.Find(origin));
                for (const auto vertexIndex : equivalentVertices) {
                    WalkOutgoingEdges(
                        edgeList, vertexIndex, [&numIncidentEdges, &equivalentVertices, &edgeList](const auto e) {
                            if (!equivalentVertices.contains(edgeList.GetDestinationIndex(e))) {
                                ++numIncidentEdges;
                            }
                        }
                    );
                }

                if (numIncidentEdges == 2) {
                    std::vector<DoublyConnectedEdgeList::edge_index> edges;
                    for (const auto vertexIndex : equivalentVertices) {
                        WalkOutgoingEdges(
                            edgeList, vertexIndex, [&edges, &equivalentVertices, &edgeList](const auto e) {
                                if (!equivalentVertices.contains(edgeList.GetDestinationIndex(e))) {
                                    edges.push_back(e);
                                }
                            }
                        );
                    }

                    COMPG_ASSERT(edges.size() == 2, "Oops");
                    const auto v0 = edgeList.GetEdgeAsVector(edges.at(0));
                    const auto v1 = edgeList.GetEdgeAsVector(edges.at(1));

                    const auto angle = Angle360(v0, v1);
                    if (math::IsZero(angle - math::PI, epsilon)) {
                        ee = edgeList.GetTwinIndex(edges.at(0) == ee ? edges.at(1) : edges.at(0));
                    } else {
                        done = true;
                    }

                } else {
                    done = true;
                }
            } while (!done);

            return ee;
        };

        DoublyConnectedEdgeList result;
        WalkUndirectedEdges(edgeList, [&](const DoublyConnectedEdgeList::edge_index edgeIndex) {
            const auto origin = edgeList.GetOriginIndex(edgeIndex);
            const auto destination = edgeList.GetDestinationIndex(edgeIndex);
            if (!visited.contains(edgeIndex) && vertexSetUnion.Find(origin) != vertexSetUnion.Find(destination)) {
                const auto e1 = goBackwards(edgeIndex);
                const auto e2 = goBackwards(edgeList.GetTwinIndex(edgeIndex));

                const auto v1 = vertexSetUnion.Find(edgeList.GetOriginIndex(e1));
                const auto v2 = vertexSetUnion.Find(edgeList.GetOriginIndex(e2));

                const auto i1 = result.InsertVertex(edgeList.GetVertex(v1).Vertex);
                const auto i2 = result.InsertVertex(edgeList.GetVertex(v2).Vertex);

                result.InsertEdge(i1, i2);
            }
        });

        UpdateFaces(result);
        return result;
    }

    CollapseVerticesResult CollapseVertices(const DoublyConnectedEdgeList& edgeList, Float epsilon) {
        const VertexEquivalenceCalculator equivalence{epsilon};
        DisjointSetUnion vertexSetUnion = equivalence.FindEquivalenceClasses(
            std::views::iota(0UZ, edgeList.NumVertices())
            | std::views::transform([&edgeList](const auto i) { return edgeList.GetVertex(i).Vertex; })
            | std::ranges::to<std::vector>()
        );

        DoublyConnectedEdgeList result;
        std::unordered_map<DoublyConnectedEdgeList::vertex_index, DoublyConnectedEdgeList::vertex_index> vertexMap;
        for (std::size_t i = 0; i < edgeList.NumVertices(); ++i) {
            const auto representative = vertexSetUnion.Find(i);
            vertexMap[i] = result.InsertVertex(edgeList.GetVertex(representative).Vertex);
        }

        for (std::size_t i = 0; i < edgeList.NumEdges(); ++i) {
            const auto v0 = vertexMap.at(edgeList.GetOriginIndex(i));
            const auto v1 = vertexMap.at(edgeList.GetDestinationIndex(i));
            if (v0 != v1) {
                result.InsertEdge(v0, v1);
            }
        }

        return {.EdgeList = result, .VertexMap = vertexMap};
    }

    std::optional<DoublyConnectedEdgeList::edge_index> FindNextOnLine(
        const DoublyConnectedEdgeList& edgeList, DoublyConnectedEdgeList::edge_index edgeIndex, Float epsilon
    ) {
        const auto edgeVector = edgeList.GetEdgeAsVector(edgeIndex);
        const auto outgoingEdges = FindOutgoingEdges(edgeList, edgeList.GetDestinationIndex(edgeIndex));
        const auto angles = outgoingEdges | std::views::transform([&edgeList, &edgeVector](const auto e) {
                                return Angle180(edgeVector, edgeList.GetEdgeAsVector(e));
                            })
                            | std::ranges::to<std::vector>();
        const auto isZero = [epsilon](const auto angle) { return math::IsZero(angle, epsilon); };
        const auto it = std::ranges::find_if(angles, isZero);
        if (it == angles.end()) {
            return std::nullopt;
        }
        const auto it2 = std::find_if(std::next(it), angles.end(), isZero);
        COMPG_ASSERT(it2 == angles.end(), "Expected only one adjacent edge with angle zero");
        return outgoingEdges.at(std::distance(angles.begin(), it));
    }
} // namespace compg