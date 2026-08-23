#include "visibility_graph/VisibilityCalculator.hpp"
#include "common/Algorithms.hpp"
#include "data_structures/AvlTree.hpp"
#include "math/Geometry.hpp"
#include "visibility_graph/HalfLineComparator.hpp"

#include <ranges>

#include "visibility_graph/Common.hpp"

namespace compg {
    using adjacency_map_type = std::unordered_map<Vertex2D, std::tuple<Vertex2D, Vertex2D>>;

    namespace details {
        auto CreateAdjacencyMap(const std::vector<Polygon>& polygons) {
            adjacency_map_type adjacencyMap;

            std::ranges::for_each(polygons, [&adjacencyMap](const Polygon& polygon) {
                std::ranges::for_each(
                    std::views::iota(0UZ, polygon.NumVertices()), [&polygon, &adjacencyMap](const auto i) {
                        const auto& vertex = polygon.GetVertex(i);
                        COMPG_ASSERT(!adjacencyMap.contains(vertex), "Expected unique vertices in the polygons");

                        const auto prevIndex = i == 0 ? polygon.NumVertices() - 1 : i - 1;
                        const auto& prev = polygon.GetVertex(prevIndex);
                        const auto nextIndex = (i + 1) % polygon.NumVertices();
                        const auto& next = polygon.GetVertex(nextIndex);
                        adjacencyMap.insert(std::tuple{vertex, std::tuple{prev, next}});
                    }
                );
            });

            return adjacencyMap;
        }

        auto CreateInitialTree(const HalfLine2D& initialHalfLine, const std::vector<Polygon>& polygons, Float epsilon) {
            AvlTree<LineSegment2D> tree;
            std::ranges::for_each(
                polygons, [comparator = HalfLineComparator{initialHalfLine, epsilon}, &initialHalfLine, epsilon,
                           &tree](const auto& polygon) {
                    for (std::size_t i = 0; i < polygon.NumEdges(); ++i) {
                        const auto edge = polygon.GetEdge(i);
                        if (const auto intersection = FindIntersection(initialHalfLine, edge);
                            intersection && !AreEqual(initialHalfLine.GetOrigin(), intersection.value(), epsilon)) {

                            const auto side0 = FindSide(initialHalfLine, edge[0]);
                            const auto side1 = FindSide(initialHalfLine, edge[1]);

                            const auto insertEdge = (side0 == PointSide::Negative && side1 == PointSide::Positive)
                                                    || (side0 == PointSide::Positive && side1 == PointSide::Negative)
                                                    || (side0 == PointSide::Negative && side1 == PointSide::Collinear)
                                                    || (side0 == PointSide::Collinear && side1 == PointSide::Negative);
                            if (insertEdge) {
                                tree.Insert(edge, comparator);
                            }
                        }
                    }
                }
            );

            return tree;
        }
    } // namespace details

    struct VisibilityState {
        adjacency_map_type AdjacencyMap;
        AvlTree<LineSegment2D> Segments;
        std::vector<Vertex2D> VisibleVertices;
    };

    bool IsVisible(
        const Vertex2D& vertex, const HalfLine2D& halfLine, const VisibilityState& state,
        const std::optional<Vertex2D>& previousVertex, Float epsilon
    ) {
        const auto [prev, next] = state.AdjacencyMap.at(vertex);
        if (EntersPolygon(halfLine.GetOrigin(), prev, vertex, next, epsilon)) {
            return false;
        }
        const auto& origin = halfLine.GetOrigin();
        /*const auto originAngle = Angle360(next - vertex, origin - vertex);
        const auto prevAngle = Angle360(next - vertex, prev - vertex);
        if (epsilon < originAngle && originAngle + epsilon < prevAngle) {
            return false;
        }*/

        if (previousVertex && FindSide(halfLine, previousVertex.value(), epsilon) == PointSide::Collinear) {
            if (!state.VisibleVertices.empty() && state.VisibleVertices.back() == previousVertex.value()) {
                const auto it
                    = state.Segments.UpperBound(previousVertex.value(), HalfLineComparator{halfLine, epsilon});

                if (it != state.Segments.end()) {
                    const auto intersection = FindIntersection(halfLine, *it, epsilon);
                    COMPG_ASSERT(intersection.has_value(), "Expected the line segment to intersect the half line");
                    const auto segmentDistance = (origin - intersection.value()).norm();
                    const auto vertexDistance = (origin - vertex).norm();
                    return vertexDistance <= segmentDistance + epsilon;
                }
                return true;
            }
            return false;
        }
        const auto it = state.Segments.begin();
        return !(
            it != state.Segments.end()
            && FindIntersection(*it, LineSegment2D{origin, vertex})
                   .transform([epsilon, &vertex](const auto& i) { return !AreEqual(i, vertex, epsilon); })
                   .value_or(false)
        );
    }

    std::vector<Vertex2D>
    VisibilityCalculator::FindVisibleVertices(const Vertex2D& origin, const std::vector<Polygon>& obstacles) const {
        if (obstacles.empty()) {
            return {};
        }
        const auto adjacencyMap = details::CreateAdjacencyMap(obstacles);
        const auto vertices
            = adjacencyMap | std::views::keys
              | std::views::filter([&origin, this](const auto& vertex) { return !AreEqual(vertex, origin, Epsilon); })
              | std::ranges::to<std::vector>();
        const auto angles = vertices | std::views::transform([&origin](const auto& vertex) {
                                const auto angle = Angle360(vertex - origin, {1, 0});
                                const auto distance = (vertex - origin).norm();
                                return Vertex2D{angle, distance};
                            })
                            | std::ranges::to<std::vector>();

        const auto argSort = ArgSort(angles, LexicographicalLess<Vertex2D>{});
        VisibilityState state{
            .AdjacencyMap = adjacencyMap,
            // direction should be {1, 0}
            .Segments
            = details::CreateInitialTree(HalfLine2D{origin, vertices.at(argSort.at(0)) - origin}, obstacles, Epsilon),
            .VisibleVertices = {}
        };

        for (const auto [i, vertexIndex] : argSort | std::views::enumerate) {
            const auto previousVertex = i > 0 ? std::optional{vertices.at(argSort.at(i - 1))} : std::nullopt;
            const auto& vertex = vertices.at(vertexIndex);
            const HalfLine2D halfLine{origin, vertex - origin};
            const HalfLineComparator comparator{halfLine, Epsilon};
            if (IsVisible(vertex, halfLine, state, previousVertex, Epsilon)) {
                state.VisibleVertices.push_back(vertex);
            }

            const auto [prev, next] = adjacencyMap.at(vertex);
            if (FindSide(halfLine, prev, Epsilon) == PointSide::Positive) {
                state.Segments.Insert(LineSegment2D{vertex, prev}, comparator);
            }
            if (FindSide(halfLine, next, Epsilon) == PointSide::Positive) {
                state.Segments.Insert(LineSegment2D{vertex, next}, comparator);
            }

            const auto upperBound = state.Segments.UpperBound(vertex, comparator);
            for (auto it = state.Segments.LowerBound(vertex, comparator); it != upperBound;) {
                const auto nextIt = std::next(it);
                const auto& segment = *it;
                if (FindSide(halfLine, segment[0], Epsilon) == PointSide::Negative
                    || FindSide(halfLine, segment[1], Epsilon) == PointSide::Negative) {
                    state.Segments.Erase(it);
                }
                it = nextIt;
            }
        }

        return state.VisibleVertices;
    }
} // namespace compg
