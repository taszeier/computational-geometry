#include "visibility_graph/ShortestPointPathCalculator.hpp"
#include "data_structures/graph/GraphAlgorithms.hpp"
#include "math/Geometry.hpp"
#include "motion_planning/FreeSpaceCalculator.hpp"
#include "visibility_graph/Common.hpp"
#include "visibility_graph/VisibilityCalculator.hpp"

namespace compg {
    namespace details {
        /**
         * @brief Find whether two vertices in a box are on the same edge of the box.
         */
        bool AreOnTheSameEdge(const Box2D& box, const Vertex2D& v0, const Vertex2D& v1, Float epsilon = EPSILON) {
            const auto corners = GetCorners(box);
            for (std::size_t i = 0; i < corners.size(); ++i) {
                std::size_t j = (i + 1) % corners.size();
                const auto& c0 = corners.at(i);
                const auto& c1 = corners.at(j);
                if (FindSide(c0, c1, v0, epsilon) == PointSide::Collinear
                    && FindSide(c0, c1, v1, epsilon) == PointSide::Collinear) {
                    return true;
                }
            }
            return false;
        }

        template <typename GraphType>
        void ConnectToVisibleVertices(
            const Vertex2D& vertex, const std::vector<Polygon>& polygons, const Box2D& box, GraphType& graph,
            Float epsilon = EPSILON
        ) {
            const VisibilityCalculator calculator{epsilon};
            const auto visibleVertices = calculator.FindVisibleVertices(vertex, polygons);
            for (const auto& visibleVertex : visibleVertices) {
                // TODO: check if the segment from vertex to visibleVertex is not in the interior of the polygons after
                // joined with the "unbounded" polygon This is just a hack.
                if (!AreOnTheSameEdge(box, vertex, visibleVertex, epsilon)) {
                    const auto weight = (visibleVertex - vertex).norm();
                    graph.InsertEdge(typename GraphType::edge_type{vertex, visibleVertex}, weight);
                }
            }
        }

        bool Contains(const LineSegment2D& segment, const Vertex2D& vertex, Float epsilon = EPSILON) {
            const auto side = FindSide(segment, vertex, epsilon);
            if (side != PointSide::Collinear) {
                return false;
            }
            const Vertex2D direction = segment[1] - segment[0];
            const Vertex2D normal{direction[1], -direction[0]};
            if (FindSide(segment[0], segment[0] + normal, vertex, epsilon) == PointSide::Positive) {
                return false;
            }
            return FindSide(segment[1], segment[1] + normal, vertex, epsilon) != PointSide::Negative;
        }

        bool ContainsPolygonEdge(
            const LineSegment2D& segment, const std::vector<Polygon>& polygons, Float epsilon = EPSILON
        ) {
            for (const auto& polygon : polygons) {
                for (std::size_t i = 0; i < polygon.NumEdges(); ++i) {
                    const auto edge = polygon.GetEdge(i);
                    if (Contains(segment, edge[0], epsilon) && Contains(segment, edge[1], epsilon)) {
                        return true;
                    }
                }
            }
            return false;
        }

        bool AreVisible(
            const Vertex2D& v0, const Vertex2D& v1, const Box2D& box, const std::vector<Polygon>& polygons,
            Float epsilon = EPSILON
        ) {
            const LineSegment2D segment{v0, v1};

            if (AreOnTheSameEdge(box, v0, v1, epsilon) && ContainsPolygonEdge(segment, polygons, epsilon)) {
                return false;
            }

            for (const auto& polygon : polygons) {
                for (std::size_t j = 0; j < polygon.NumVertices(); ++j) {
                    const auto i = (j + (polygon.NumVertices() - 1)) % polygon.NumVertices();
                    const auto k = (j + 1) % polygon.NumVertices();
                    const LineSegment2D edge{polygon.GetVertex(j), polygon.GetVertex(k)};
                    const auto maybeIntersection = FindIntersection(edge, segment, epsilon);

                    const auto intersectsProperly = [&v0, &v1, &edge, epsilon](const auto& intersection) {
                        if (!AreEqual(intersection, edge[0], epsilon) && !AreEqual(intersection, edge[1], epsilon)) {
                            if (AreEqual(intersection, v0, epsilon)) {
                                return FindSide(edge, v1, epsilon) == PointSide::Negative;
                            }
                            if (AreEqual(intersection, v1, epsilon)) {
                                return FindSide(edge, v0, epsilon) == PointSide::Negative;
                            }
                            return true;
                        }
                        return false;
                        /*return !AreEqual(intersection, v0, epsilon) && !AreEqual(intersection, v1, epsilon)
                               && !AreEqual(intersection, edge[0], epsilon)
                               && !AreEqual(intersection, edge[1], epsilon);*/
                    };
                    const auto entersPolygon = [i, j, k, epsilon, &polygon, &v0, &v1](const auto& intersection) {
                        const auto& vi = polygon.GetVertex(i);
                        const auto& vj = polygon.GetVertex(j);
                        const auto& vk = polygon.GetVertex(k);

                        if (AreEqual(intersection, vj, epsilon)) {
                            if (AreEqual(vj, v0, epsilon)) {
                                return EntersPolygon(v1, vi, vj, vk, epsilon);
                            }
                            if (AreEqual(vj, v1, epsilon)) {
                                return EntersPolygon(v0, vi, vj, vk, epsilon);
                            }
                            const auto side_i = FindSide(v0, v1, vi, epsilon);
                            const auto side_k = FindSide(v0, v1, vk, epsilon);
                            return (side_i == PointSide::Negative && side_k == PointSide::Positive)
                                   || (side_i == PointSide::Positive && side_k == PointSide::Negative);
                        }
                        return false;
                    };

                    if (maybeIntersection
                            .transform([&intersectsProperly, &entersPolygon](const auto& intersection) {
                                return intersectsProperly(intersection) || entersPolygon(intersection);
                            })
                            .value_or(false)) {
                        return false;
                    }
                }
            }

            return true;
        }
    } // namespace details

    ShortestPointPathCalculator::ShortestPointPathCalculator(
        const Box2D& box, const std::vector<Polygon>& obstacles, std::size_t seed
    )
        : FreeSpace(FreeSpaceCalculator{}.FindFreeSpace(box, obstacles, seed))
        , Obstacles(obstacles) {

        const auto vertices = GetVertices(obstacles);
        for (const auto& vertex : vertices) {
            const bool inserted = VisibilityGraph.InsertNode(vertex);
            COMPG_ASSERT(inserted, "Expected the polygons to have unique vertices");
        }

        for (const auto& vertex : vertices) {
            details::ConnectToVisibleVertices(vertex, obstacles, FreeSpace.Box, VisibilityGraph);
        }
    }

    std::optional<std::vector<Vertex2D>>
    ShortestPointPathCalculator::FindPath(const Vertex2D& start, const Vertex2D& goal) const {
        if (!Contains(FreeSpace, start) || !Contains(FreeSpace, goal)) {
            return std::nullopt;
        }

        if (AreEqual(start, goal)) {
            return {{goal}};
        }
        if (details::AreVisible(start, goal, FreeSpace.Box, Obstacles)) {
            return {{start, goal}};
        }

        auto visibilityGraph{VisibilityGraph};
        if (!visibilityGraph.HasNode(start)) {
            visibilityGraph.InsertNode(start);
            details::ConnectToVisibleVertices(start, Obstacles, FreeSpace.Box, visibilityGraph);
        }
        if (!visibilityGraph.HasNode(goal)) {
            visibilityGraph.InsertNode(goal);
            details::ConnectToVisibleVertices(goal, Obstacles, FreeSpace.Box, visibilityGraph);
        }

        const DijkstrasAlgorithm dijkstra;
        return dijkstra.FindShortestPath(visibilityGraph, start, goal);
    }
} // namespace compg