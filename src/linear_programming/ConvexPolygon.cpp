#include "linear_programming/ConvexPolygon.hpp"

namespace compg {

    std::optional<Vertex2D> FindTopVertex(const ConvexPolygon& polygon) {
        COMPG_ASSERT(!(polygon.LeftBoundary.empty() && polygon.RightBoundary.empty()), "Expected a non-empty polygon");

        if (!polygon.LeftBoundary.empty() && !polygon.RightBoundary.empty()) {

            const auto& left = polygon.LeftBoundary.at(0);
            const auto& right = polygon.RightBoundary.at(0);
            if (FindBoundaryRelation(left, right) == BoundaryRelation::BoundedAbove) {
                const auto intersection = FindIntersection(left, right);
                COMPG_ASSERT(intersection.has_value(), "Expected the boundaries to intersect");
                return intersection.value();
            }
            return std::nullopt;
        }
        return std::nullopt;
    }

    std::optional<Vertex2D> FindBottomVertex(const ConvexPolygon& polygon) {
        COMPG_ASSERT(!(polygon.LeftBoundary.empty() && polygon.RightBoundary.empty()), "Expected a non-empty polygon");

        if (!polygon.LeftBoundary.empty() && !polygon.RightBoundary.empty()) {

            const auto& left = polygon.LeftBoundary.at(0);
            const auto& right = polygon.RightBoundary.at(0);
            if (FindBoundaryRelation(left, right) == BoundaryRelation::BoundedBelow) {
                const auto intersection = FindIntersection(left, right);
                COMPG_ASSERT(intersection.has_value(), "Expected the boundaries to intersect");
                return intersection.value();
            }
            return std::nullopt;
        }
        return std::nullopt;
    }

    std::optional<std::size_t>
    FindIntersectionBoundary(const auto& boundary, const std::optional<Vertex2D>& topVertex) {
        if (boundary.empty()) {
            return std::nullopt;
        }
        if (!topVertex.has_value()) {
            return 0;
        }

        // boundary | std::views::slide(2) | std::views::enumerate
        for (std::size_t i = 0; i + 1 < boundary.size(); ++i) {
            const auto intersection = FindIntersection(boundary.at(i), boundary.at(i + 1));
            COMPG_ASSERT(intersection.has_value(), "Expected the boundaries to intersect");
            if (VertexBelowComparator::Compare(intersection.value(), topVertex.value())) {
                return std::optional{i};
            }
        }
        return std::optional{boundary.size() - 1};
    }

    ConvexPolygon::Edge::boundary_edge_type CreateBoundaryEdge(
        const std::optional<Vertex2D>& upperEndpoint, const std::optional<Vertex2D>& lowerEndpoint,
        const Hyperplane<2UZ>& boundary
    ) {
        if (upperEndpoint.has_value() && lowerEndpoint.has_value()) {
            return LineSegment2D{upperEndpoint.value(), lowerEndpoint.value()};
        }
        if (!upperEndpoint.has_value() && !lowerEndpoint.has_value()) {
            return ConvertTo<Line2D>(boundary);
        }

        Vertex2D normal{-boundary.GetNormal()[1], boundary.GetNormal()[0]};
        const auto angle = Angle360({1, 0}, normal);
        if (lowerEndpoint.has_value()) {
            if (angle == 0 || angle > math::PI) {
                normal *= -1;
            }
            return HalfLine2D{lowerEndpoint.value(), normal};
        }
        if (0 < angle && angle <= math::PI) {
            normal *= -1;
        }
        return HalfLine2D{upperEndpoint.value(), normal};
    }

    std::optional<ConvexPolygon::Edge>
    FindLeftIntersectionEdge(const ConvexPolygon& polygon, const std::optional<Vertex2D>& vertex) {

        const auto& boundary = polygon.LeftBoundary;
        const auto& otherBoundary = polygon.RightBoundary;
        const auto boundaryIndex = FindIntersectionBoundary(boundary, vertex);
        if (!boundaryIndex.has_value()) {
            return std::nullopt;
        }
        const auto& b = polygon.LeftBoundary.at(boundaryIndex.value());

        if (boundary.size() == 1) {
            if (otherBoundary.empty()) {
                return ConvexPolygon::Edge{CreateBoundaryEdge(std::nullopt, std::nullopt, b),
                                           std::nullopt,
                                           std::nullopt,
                                           boundaryIndex.value(),
                                           polygon,
                                           true};
            }
            if (otherBoundary.size() == 1) {
                const auto x = FindBoundaryRelation(polygon.LeftBoundary.at(0), polygon.RightBoundary.at(0));
                if (x == BoundaryRelation::Parallel) {
                    return ConvexPolygon::Edge{CreateBoundaryEdge(std::nullopt, std::nullopt, b),
                                               std::nullopt,
                                               std::nullopt,
                                               boundaryIndex.value(),
                                               polygon,
                                               true};
                }
                const auto intersection = FindIntersection(polygon.LeftBoundary.at(0), otherBoundary.at(0));
                COMPG_ASSERT(intersection.has_value(), "Expected the boundaries to intersect");
                if (x == BoundaryRelation::BoundedAbove) {
                    if (!vertex.has_value() || VertexAboveComparator::Compare(vertex.value(), intersection.value())) {
                        return std::nullopt;
                    }
                    return ConvexPolygon::Edge{
                        CreateBoundaryEdge(intersection, std::nullopt, b),
                        intersection.value(),
                        std::nullopt,
                        boundaryIndex.value(),
                        polygon,
                        true
                    };
                }
                // if (x == BoundaryReation::BoundedBelow) {
                if (vertex.has_value() && VertexBelowComparator::Compare(vertex.value(), intersection.value())) {
                    return std::nullopt;
                }
                return ConvexPolygon::Edge{
                    CreateBoundaryEdge(std::nullopt, intersection, b),
                    std::nullopt,
                    intersection.value(),
                    boundaryIndex.value(),
                    polygon,
                    true
                };
            }

            const auto upperEndpoint = FindIntersection(polygon.LeftBoundary.at(0), otherBoundary.at(0));
            const auto lowerEndpoint = FindIntersection(polygon.LeftBoundary.at(0), otherBoundary.back());
            return ConvexPolygon::Edge{
                CreateBoundaryEdge(upperEndpoint, lowerEndpoint, b),
                upperEndpoint,
                lowerEndpoint,
                boundaryIndex.value(),
                polygon,
                true
            };
        } else {
            const auto upperEndpoint
                = boundaryIndex.value() == 0
                      ? (otherBoundary.empty()
                             ? std::optional<Vertex2D>{}
                             : FindIntersection(boundary.at(boundaryIndex.value()), otherBoundary.at(0)))
                      : FindIntersection(boundary.at(boundaryIndex.value()), boundary.at(boundaryIndex.value() - 1));

            const auto lowerEndpoint
                = boundaryIndex.value() + 1 == boundary.size()
                      ? (otherBoundary.empty()
                             ? std::optional<Vertex2D>{}
                             : FindIntersection(boundary.at(boundaryIndex.value()), otherBoundary.back()))
                      : FindIntersection(boundary.at(boundaryIndex.value()), boundary.at(boundaryIndex.value() + 1));

            return ConvexPolygon::Edge{
                CreateBoundaryEdge(upperEndpoint, lowerEndpoint, b),
                upperEndpoint,
                lowerEndpoint,
                boundaryIndex.value(),
                polygon,
                true
            };
        }
    }

    std::optional<ConvexPolygon::Edge>
    FindRightIntersectionEdge(const ConvexPolygon& polygon, const std::optional<Vertex2D>& vertex) {

        const auto boundaryIndex = FindIntersectionBoundary(polygon.RightBoundary, vertex);
        if (!boundaryIndex.has_value()) {
            return std::nullopt;
        }

        const auto& b = polygon.RightBoundary.at(boundaryIndex.value());
        if (polygon.RightBoundary.size() == 1) {
            if (polygon.LeftBoundary.size() == 0) {
                return ConvexPolygon::Edge{CreateBoundaryEdge(std::nullopt, std::nullopt, b),
                                           std::nullopt,
                                           std::nullopt,
                                           boundaryIndex.value(),
                                           polygon,
                                           false};
            }
            if (polygon.LeftBoundary.size() == 1) {
                const auto x = FindBoundaryRelation(polygon.LeftBoundary.at(0), polygon.RightBoundary.at(0));
                if (x == BoundaryRelation::Parallel) {
                    return ConvexPolygon::Edge{CreateBoundaryEdge(std::nullopt, std::nullopt, b),
                                               std::nullopt,
                                               std::nullopt,
                                               boundaryIndex.value(),
                                               polygon,
                                               false};
                }
                const auto intersection = FindIntersection(polygon.LeftBoundary.at(0), polygon.RightBoundary.at(0));
                COMPG_ASSERT(intersection.has_value(), "Expected the boundaries to intersect");
                if (x == BoundaryRelation::BoundedAbove) {
                    if (!vertex.has_value() || VertexAboveComparator::Compare(vertex.value(), intersection.value())) {
                        return std::nullopt;
                    }
                    return ConvexPolygon::Edge{
                        CreateBoundaryEdge(intersection, std::nullopt, b),
                        intersection.value(),
                        std::nullopt,
                        boundaryIndex.value(),
                        polygon,
                        false
                    };
                }
                // if (x == BoundaryReation::BoundedBelow) {
                if (vertex.has_value() && VertexBelowComparator::Compare(vertex.value(), intersection.value())) {
                    return std::nullopt;
                }
                return ConvexPolygon::Edge{
                    CreateBoundaryEdge(std::nullopt, intersection, b),
                    std::nullopt,
                    intersection.value(),
                    boundaryIndex.value(),
                    polygon,
                    false
                };
            }

            const auto upperEndpoint = FindIntersection(polygon.RightBoundary.at(0), polygon.LeftBoundary.at(0));
            const auto lowerEndpoint = FindIntersection(polygon.RightBoundary.at(0), polygon.LeftBoundary.back());
            return ConvexPolygon::Edge{
                CreateBoundaryEdge(upperEndpoint, lowerEndpoint, b),
                upperEndpoint,
                lowerEndpoint,
                boundaryIndex.value(),
                polygon,
                false
            };
        } else {
            const auto upperEndpoint
                = boundaryIndex.value() == 0
                      ? (polygon.LeftBoundary.empty()
                             ? std::optional<Vertex2D>{}
                             : FindIntersection(
                                   polygon.RightBoundary.at(boundaryIndex.value()), polygon.LeftBoundary.at(0)
                               ))
                      : FindIntersection(
                            polygon.RightBoundary.at(boundaryIndex.value()),
                            polygon.RightBoundary.at(boundaryIndex.value() - 1)
                        );

            const auto lowerEndpoint
                = boundaryIndex.value() + 1 == polygon.RightBoundary.size()
                      ? (polygon.LeftBoundary.empty()
                             ? std::optional<Vertex2D>{}
                             : FindIntersection(
                                   polygon.RightBoundary.at(boundaryIndex.value()), polygon.LeftBoundary.back()
                               ))
                      : FindIntersection(
                            polygon.RightBoundary.at(boundaryIndex.value()),
                            polygon.RightBoundary.at(boundaryIndex.value() + 1)
                        );

            return ConvexPolygon::Edge{
                CreateBoundaryEdge(upperEndpoint, lowerEndpoint, b),
                upperEndpoint,
                lowerEndpoint,
                boundaryIndex.value(),
                polygon,
                false
            };
        }
    }

    void GoToNextEdge(ConvexPolygon::Edge& edge) {

        const auto& boundary = edge.GetBoundary();
        const auto& oppositeBoundary = edge.GetOppositeBoundary();
        COMPG_ASSERT(edge.BoundaryIndex + 1 < boundary.size(), "The edge has no successor");

        const auto nextEdgeBoundaryIndex = edge.BoundaryIndex + 1;

        const auto lowerEndpoint
            = nextEdgeBoundaryIndex + 1 == boundary.size()
                  ? (oppositeBoundary.empty()
                         ? std::optional<Vertex2D>{}
                         : FindIntersection(boundary.at(nextEdgeBoundaryIndex), oppositeBoundary.back()))
                  : FindIntersection(boundary.at(nextEdgeBoundaryIndex), boundary.at(nextEdgeBoundaryIndex + 1));

        edge.UpperEndpoint = edge.LowerEndpoint;
        COMPG_ASSERT(edge.UpperEndpoint.has_value(), "Oops");
        edge.LowerEndpoint
            = (lowerEndpoint.has_value()
               && LexicographicalLess<Vertex2D>::Compare(edge.UpperEndpoint.value(), lowerEndpoint.value()))
                  ? std::nullopt
                  : lowerEndpoint;
        edge.BoundaryIndex = nextEdgeBoundaryIndex;
        edge.BoundaryEdge = CreateBoundaryEdge(edge.UpperEndpoint, edge.LowerEndpoint, edge.GetBoundaryHalfPlane());
    }

} // namespace compg