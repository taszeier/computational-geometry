#include "triangulation/PolygonMonotonizer.hpp"
#include "common/Vertex.hpp"
#include "sweep_line/SweepLine.hpp"

#include <ranges>
#include <set>

#include "data_structures/AvlTree.hpp"
#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include "math/Geometry.hpp"

namespace compg {

    using vertex_index = DoublyConnectedEdgeList::vertex_index;
    using edge_index = DoublyConnectedEdgeList::edge_index;

    namespace details {
        std::vector<vertex_index> CreateEventPoints(const DoublyConnectedEdgeList& polygon) {
            std::vector<vertex_index> eventPoints(polygon.NumVertices());
            std::ranges::iota(eventPoints, 0);
            std::ranges::sort(eventPoints, VertexAboveComparator{}, [&polygon](vertex_index i) {
                return polygon.GetVertex(i).Vertex;
            });
            return eventPoints;
        }

        enum class VertexType {
            Start,
            End,
            Regular,
            Split,
            Merge
        };

        edge_index FindLeftEdge(vertex_index startIndex, const DoublyConnectedEdgeList& polygon) {
            const auto edges = FindOutgoingEdges(polygon, startIndex);
            COMPG_ASSERT(edges.size() == 2, "Expected the start vertex to have two incident edges");
            const auto upVector = CreateCanonicalVector<2>(1);
            return *std::ranges::min_element(edges, Less{}, [&polygon, &upVector](edge_index edgeIndex) {
                const auto v = polygon.GetEdgeAsVector(edgeIndex);
                return Angle360(upVector, v);
            });
        }

        edge_index
        FindLeftEdge(vertex_index currentIndex, vertex_index previousIndex, const DoublyConnectedEdgeList& polygon) {
            const auto edges = FindOutgoingEdges(polygon, currentIndex);
            COMPG_ASSERT(edges.size() == 2, "Expected the vertex to have two incident edges");
            const auto previousEdgeIt = std::ranges::find_if(edges, [&polygon, previousIndex](edge_index e) {
                return polygon.GetDestinationIndex(e) == previousIndex;
            });
            COMPG_ASSERT(previousEdgeIt != edges.end(), "Could not find the edge to the previous vertex");
            const auto previousEdgeVecIdx = std::distance(edges.begin(), previousEdgeIt);
            const auto nextEdgeVecIdx = (previousEdgeVecIdx + 1) % 2;

            return edges.at(nextEdgeVecIdx);
        }

        std::vector<edge_index> FindLeftEdges(vertex_index startIndex, const DoublyConnectedEdgeList& polygon) {
            std::vector<edge_index> leftEdges(polygon.NumVertices(), polygon.NumEdges());
            const auto startVertexLeftEdge = FindLeftEdge(startIndex, polygon);
            leftEdges.at(startIndex) = startVertexLeftEdge;

            vertex_index previousVertexIndex = startIndex;
            edge_index previousEdgeIndex = startVertexLeftEdge;
            vertex_index currentVertexIndex = polygon.GetDestinationIndex(previousEdgeIndex);

            for ([[maybe_unused]] std::size_t i = 1; i < polygon.NumVertices(); ++i) {
                const auto nextEdgeIndex = FindLeftEdge(currentVertexIndex, previousVertexIndex, polygon);
                leftEdges[currentVertexIndex] = nextEdgeIndex;
                const auto nextVertexIndex = polygon.GetDestinationIndex(nextEdgeIndex);

                previousVertexIndex = currentVertexIndex;
                previousEdgeIndex = nextEdgeIndex;
                currentVertexIndex = nextVertexIndex;
            }

            return leftEdges;
        }

        VertexType FindVertexType(
            vertex_index previousVertexIndex, vertex_index currentVertexIndex, vertex_index nextVertexIndex,
            edge_index previousEdgeIndex, edge_index nextEdgeIndex, const DoublyConnectedEdgeList& polygon
        ) {

            const auto previousEdgeVector = polygon.GetEdgeAsVector(polygon.GetTwinIndex(previousEdgeIndex));
            const auto nextEdgeVector = polygon.GetEdgeAsVector(nextEdgeIndex);
            const auto interiorAngle = Angle360(nextEdgeVector, previousEdgeVector);

            const auto previousVertex = polygon.GetVertex(previousVertexIndex).Vertex;
            const auto currentVertex = polygon.GetVertex(currentVertexIndex).Vertex;
            const auto nextVertex = polygon.GetVertex(nextVertexIndex).Vertex;

            if (VertexBelowComparator::Compare(previousVertex, currentVertex)
                && VertexBelowComparator::Compare(nextVertex, currentVertex)) {
                return interiorAngle < math::PI ? VertexType::Start : VertexType::Split;
            }
            if (VertexAboveComparator::Compare(previousVertex, currentVertex)
                && VertexAboveComparator::Compare(nextVertex, currentVertex)) {
                return interiorAngle < math::PI ? VertexType::End : VertexType::Merge;
            }
            return VertexType::Regular;
        }

        std::unordered_map<vertex_index, VertexType>
        FindVertexTypes(vertex_index startIndex, const DoublyConnectedEdgeList& polygon) {
            std::unordered_map<vertex_index, VertexType> vertexTypes;
            vertexTypes.insert(std::pair{startIndex, VertexType::Start});

            vertex_index previousVertexIndex = startIndex;
            edge_index previousEdgeIndex = FindLeftEdge(startIndex, polygon);
            vertex_index currentVertexIndex = polygon.GetDestinationIndex(previousEdgeIndex);

            for ([[maybe_unused]] std::size_t i = 1; i < polygon.NumVertices(); ++i) {
                const auto nextEdgeIndex = FindLeftEdge(currentVertexIndex, previousVertexIndex, polygon);
                const auto nextVertexIndex = polygon.GetDestinationIndex(nextEdgeIndex);

                vertexTypes.insert(
                    std::pair{
                        currentVertexIndex, FindVertexType(
                                                previousVertexIndex, currentVertexIndex, nextVertexIndex,
                                                previousEdgeIndex, nextEdgeIndex, polygon
                                            )
                    }
                );

                previousVertexIndex = currentVertexIndex;
                previousEdgeIndex = nextEdgeIndex;
                currentVertexIndex = nextVertexIndex;
            }

            return vertexTypes;
        }

        class EdgeIndexLeftToRightComparator {
        public:
            explicit EdgeIndexLeftToRightComparator(const DoublyConnectedEdgeList& polygon, const Vertex2D& eventPoint)
                : Polygon(polygon)
                , EventPoint(eventPoint)
                , Comparator(eventPoint) {}

            bool operator()(edge_index lhs, edge_index rhs) const {
                return Comparator(Polygon.GetEdgeAsLineSegment(lhs), Polygon.GetEdgeAsLineSegment(rhs));
            }

            [[nodiscard]] auto FindSweepLineIntersection(edge_index edgeIndex) const {
                const auto lineSegment = Polygon.GetEdgeAsLineSegment(edgeIndex);
                return FindIntersection(lineSegment, MakeLine2DFromY(EventPoint.y())).value_or(EventPoint);
            }

            bool operator()(edge_index edgeIndex, const Vertex2D& vertex) const {
                const auto intersection = FindSweepLineIntersection(edgeIndex);
                return intersection.x() < vertex.x();
            }

            bool operator()(const Vertex2D& vertex, edge_index edgeIndex) const {
                const auto intersection = FindSweepLineIntersection(edgeIndex);
                return vertex.x() < intersection.x();
            }

        private:
            const DoublyConnectedEdgeList& Polygon;
            Vertex2D EventPoint;
            LineSegmentLeftToRightComparator Comparator;
        };

        class MonotonizerState {
        public:
            explicit MonotonizerState(DoublyConnectedEdgeList& polygon)
                : Polygon(polygon) {}

            void Run() {
                EventPoints = CreateEventPoints(Polygon);
                LeftEdges = FindLeftEdges(EventPoints.at(0), Polygon);
                VertexTypes = FindVertexTypes(EventPoints.at(0), Polygon);

                Result.BeginVertex = EventPoints.at(0);
                Result.BeginLeftEdge = LeftEdges.at(EventPoints.at(0));

                std::ranges::for_each(EventPoints, [this](vertex_index vertexIndex) {
                    HandleVertex(vertexIndex, VertexTypes[vertexIndex]);
                });
            }

            void HandleVertex(vertex_index vertexIndex, VertexType vertexType) {
                switch (vertexType) {
                case VertexType::Start:
                    return HandleStartVertex(vertexIndex);
                case VertexType::End:
                    return HandleEndVertex(vertexIndex);
                case VertexType::Split:
                    return HandleSplitVertex(vertexIndex);
                case VertexType::Merge:
                    return HandleMergeVertex(vertexIndex);
                case VertexType::Regular:
                    return HandleRegularVertex(vertexIndex);
                }

                COMPG_THROW("Received unexpected value for vertex type");
            }

            void HandleStartVertex(vertex_index vertexIndex) {
                const auto vertex = Polygon.GetVertex(vertexIndex).Vertex;
                State.Insert(LeftEdges.at(vertexIndex), EdgeIndexLeftToRightComparator{Polygon, vertex});
                Helpers[LeftEdges.at(vertexIndex)] = vertexIndex;
            }

            void HandleEndVertex(vertex_index vertexIndex) {
                UpdatePreviousLeftEdge(vertexIndex);
            }

            void HandleSplitVertex(vertex_index vertexIndex) {
                const auto vertex = Polygon.GetVertex(vertexIndex).Vertex;
                auto it = State.LowerBound(
                    Polygon.GetVertex(vertexIndex).Vertex, EdgeIndexLeftToRightComparator{Polygon, vertex}
                );
                COMPG_ASSERT(it != State.begin(), "There is no edge to the left of the vertex");
                --it;
                InsertEdge(Helpers.at(*it), vertexIndex);
                Helpers[*it] = vertexIndex;

                State.Insert(LeftEdges.at(vertexIndex), EdgeIndexLeftToRightComparator{Polygon, vertex});
                Helpers[LeftEdges.at(vertexIndex)] = vertexIndex;
            }

            void HandleMergeVertex(vertex_index vertexIndex) {
                UpdatePreviousLeftEdge(vertexIndex);
                UpdateEdgeDirectlyLeft(vertexIndex);
            }

            void HandleRegularVertex(vertex_index vertexIndex) {
                const auto previousVertexIndex
                    = Polygon.GetOriginIndex(Polygon.GetEdge(LeftEdges.at(vertexIndex)).PreviousIndex);
                const auto nextVertexIndex = Polygon.GetDestinationIndex(LeftEdges.at(vertexIndex));
                if (VertexAboveComparator::Compare(
                        Polygon.GetVertex(previousVertexIndex).Vertex, Polygon.GetVertex(nextVertexIndex).Vertex
                    )) {
                    UpdatePreviousLeftEdge(vertexIndex);

                    const auto vertex = Polygon.GetVertex(vertexIndex).Vertex;
                    const auto leftEdge = LeftEdges.at(vertexIndex);
                    State.Insert(leftEdge, EdgeIndexLeftToRightComparator{Polygon, vertex});
                    Helpers[leftEdge] = vertexIndex;
                } else {
                    UpdateEdgeDirectlyLeft(vertexIndex);
                }
            }

            void UpdatePreviousLeftEdge(vertex_index vertexIndex) {
                const auto previousLeftEdge = Polygon.GetEdge(LeftEdges[vertexIndex]).PreviousIndex;
                const vertex_index helper = Helpers.at(previousLeftEdge);
                if (VertexTypes.at(helper) == VertexType::Merge) {
                    InsertEdge(vertexIndex, helper);
                }
                const auto vertex = Polygon.GetVertex(vertexIndex).Vertex;
                State.Erase(previousLeftEdge, EdgeIndexLeftToRightComparator{Polygon, vertex});
            }

            void UpdateEdgeDirectlyLeft(vertex_index vertexIndex) {
                const auto vertex = Polygon.GetVertex(vertexIndex).Vertex;
                auto it = State.LowerBound(
                    Polygon.GetVertex(vertexIndex).Vertex, EdgeIndexLeftToRightComparator{Polygon, vertex}
                );
                COMPG_ASSERT(it != State.begin(), "There is no edge to the left of the vertex");
                --it;
                vertex_index helper = Helpers.at(*it);
                if (VertexTypes.at(helper) == VertexType::Merge) {
                    InsertEdge(vertexIndex, helper);
                }
                Helpers[*it] = vertexIndex;
            }

            void InsertEdge(vertex_index v1, vertex_index v2) {
                const auto edgeIndex = Polygon.InsertEdge(v1, v2);
                Result.InsertedEdges.push_back(edgeIndex);
            }

            auto GetResult() const {
                return Result;
            }

        private:
            std::vector<vertex_index> EventPoints;
            std::vector<edge_index> LeftEdges;
            std::unordered_map<vertex_index, VertexType> VertexTypes;
            DoublyConnectedEdgeList& Polygon;
            AvlTree<edge_index> State;
            std::unordered_map<edge_index, vertex_index> Helpers;

            PolygonMonotonizerResult Result{};
        };
    } // namespace details

    PolygonMonotonizerResult PolygonMonotonizer::MakeMonotone(DoublyConnectedEdgeList& polygon) const {
        details::MonotonizerState state{polygon};
        state.Run();
        return state.GetResult();
    }
} // namespace compg
