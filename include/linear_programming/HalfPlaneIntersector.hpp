#pragma once

#include "common/Algorithms.hpp"
#include "linear_programming/ConvexPolygon.hpp"
#include "math/Math.hpp"
#include "math/primitives/HalfPlane.hpp"

#include <utility>
#include <variant>

namespace compg {

    namespace details {
        inline std::optional<Vertex2D> FindEdgeIntersection(
            const ConvexPolygon::Edge::boundary_edge_type& edge1, const ConvexPolygon::Edge::boundary_edge_type& edge2
        ) {
            return std::visit(
                [&edge2](const auto& e1) {
                    return std::visit([&e1](const auto& e2) { return FindIntersection(e1, e2); }, edge2);
                },
                edge1
            );
        }

        inline std::optional<Vertex2D>
        FindEdgeIntersection(const ConvexPolygon::Edge::boundary_edge_type& edge, const Line2D& line) {
            return std::visit([&line](const auto& e) { return FindIntersection(e, line); }, edge);
        }

        class FiniteEdgeLeftToRightComparator {
        public:
            explicit FiniteEdgeLeftToRightComparator(const Vertex2D& eventPoint, Float epsilon = EPSILON)
                : EventPoint(eventPoint)
                , Epsilon(epsilon) {}

        public:
            bool operator()(const auto& edge1, const auto& edge2) const {
                const Line2D sweepLine = MakeLine2DFromY(EventPoint[1]);

                const std::optional<Vertex2D> maybeIntersection1 = FindEdgeIntersection(edge1, sweepLine);
                const std::optional<Vertex2D> maybeIntersection2 = FindEdgeIntersection(edge2, sweepLine);

                const auto intersection1 = maybeIntersection1.value_or(EventPoint);
                const auto intersection2 = maybeIntersection2.value_or(EventPoint);

                return AreEqual(intersection1, intersection2, Epsilon) ? ComputeAngle(edge1) < ComputeAngle(edge2)
                                                                       : intersection1[0] < intersection2[0];
            }

        private:
            bool IsAngleLarger(const auto& edge1, const auto& edge2) const {
                return ComputeAngle(edge1) < ComputeAngle(edge2);
            }

            Float ComputeAngle(const auto& edge) const {
                const auto v = std::visit(
                    Overloads{
                        [](const auto& l) -> Vertex2D { return l[1] - l[0]; },
                        [](const HalfLine2D& hl) { return hl.GetNormal(); },
                    },
                    edge
                );
                auto angle = Angle360({1, 0}, v);
                if (angle >= math::PI) {
                    angle -= math::PI;
                }
                return angle;
            }

        private:
            Vertex2D EventPoint;
            Float Epsilon;
        };

        class InfiniteEdgeLeftToRightComparator {
        public:
            bool operator()(const auto& edge1, const auto& edge2) const {
                const auto angle1 = FindAngle(edge1);
                const auto angle2 = FindAngle(edge2);
                return angle1 == angle2 ? Comparator(edge1, edge2) : angle1 > angle2;
            }

        private:
            Float FindAngle(const auto& edge) const {
                return std::visit(
                    Overloads{
                        [](const Line2D& line) {
                            Float angle = Angle360({1, 0}, line[1] - line[0]);
                            if (math::IsZero(angle)) {
                                return math::PI;
                            }
                            if (angle > math::PI) {
                                angle -= math::PI;
                            }
                            return angle;
                        },
                        [](const HalfLine2D& halfLine) {
                            const Float angle = Angle360({1, 0}, halfLine.GetNormal());
                            COMPG_ASSERT(0 < angle && angle <= math::PI, "Oops");
                            return angle;
                        },
                        [](const LineSegment2D&) -> Float { COMPG_THROW("Oops"); }
                    },
                    edge
                );
            }

            FiniteEdgeLeftToRightComparator Comparator{{0, 0}};
        };

        class EdgeLeftToRightComparator {
        public:
            explicit EdgeLeftToRightComparator(
                std::optional<Vertex2D> eventPoint = std::nullopt, Float epsilon = EPSILON
            ) {
                if (eventPoint.has_value()) {
                    Comparator = FiniteEdgeLeftToRightComparator{eventPoint.value(), epsilon};
                } else {
                    Comparator = InfiniteEdgeLeftToRightComparator{};
                }
            }

            bool operator()(const auto& edge1, const auto& edge2) const {
                return std::visit(
                    [&edge1, &edge2](const auto& comparator) { return comparator(edge1, edge2); }, Comparator
                );
            }

        private:
            std::variant<FiniteEdgeLeftToRightComparator, InfiniteEdgeLeftToRightComparator> Comparator{
                InfiniteEdgeLeftToRightComparator{}
            };
        };

        class EdgeComparator {
        public:
            explicit EdgeComparator(ConvexPolygon::Edge edge)
                : Edge(std::move(edge)) {
                COMPG_ASSERT(Edge.UpperEndpoint.has_value(), "Oops");
            }

            bool IsRightOf(const ConvexPolygon::Edge& edge) const {
                return !IsLeftOf(edge);
            }

            bool IsLeftOf(const ConvexPolygon::Edge& edge) const {
                const Line2D sweepLine = MakeLine2DFromY(Edge.UpperEndpoint.value()[1]);

                const std::optional<Vertex2D> maybeIntersection1 = FindEdgeIntersection(Edge.BoundaryEdge, sweepLine);
                const std::optional<Vertex2D> maybeIntersection2 = FindEdgeIntersection(edge.BoundaryEdge, sweepLine);
                if (maybeIntersection1.has_value()) {
                    if (maybeIntersection2.has_value()) {
                        return FiniteEdgeLeftToRightComparator{
                            Edge.UpperEndpoint.value()
                        }(Edge.BoundaryEdge, edge.BoundaryEdge);
                    }
                    return Edge.UpperEndpoint.value()[1] < GetYCoordinate(edge);
                }
                if (maybeIntersection2.has_value()) {
                    return Edge.UpperEndpoint.value()[0] < maybeIntersection2.value()[0];
                }

                return GetYCoordinate(Edge) < GetYCoordinate(edge);
            }

            static Float GetYCoordinate(const ConvexPolygon::Edge& edge) {
                return std::visit(
                    Overloads{
                        [](const HalfLine2D& hl) -> Float { return hl.GetOrigin()[1]; },
                        [](const auto& l) -> Float { return l[0][1]; }
                    },
                    edge.BoundaryEdge
                );
            }

        private:
            ConvexPolygon::Edge Edge;
        };

        inline bool IsTopBelowPolygon(const std::optional<Vertex2D>& top, const ConvexPolygon& polygon) {
            if (top.has_value()) {
                const auto bottom = FindBottomVertex(polygon);
                return bottom.has_value() && VertexBelowComparator::Compare(top.value(), bottom.value());
            }
            return false;
        }

        std::optional<Vertex2D> FindBoundaryEndpoint(const auto& boundary, std::optional<std::size_t> halfPlaneIndex) {
            if (halfPlaneIndex.has_value() && halfPlaneIndex.value() + 1 < boundary.size()) {
                const auto& h1 = boundary.at(halfPlaneIndex.value());
                const auto& h2 = boundary.at(halfPlaneIndex.value() + 1);
                const auto intersection = FindIntersection(h1, h2);
                COMPG_ASSERT(intersection.has_value(), "Expected the boundaries to intersect");
                return intersection;
            }
            return std::nullopt;
        }

        bool IntersectsBelow(
            const ConvexPolygon::Edge& leftEdge, const std::optional<ConvexPolygon::Edge>& otherLeftEdge,
            const std::optional<Vertex2D>& eventPoint
        ) {
            if (!otherLeftEdge.has_value()) {
                return false;
            }
            const auto& edge = otherLeftEdge.value();
            return details::FindEdgeIntersection(leftEdge.BoundaryEdge, edge.BoundaryEdge)
                .transform([&eventPoint](const Vertex2D& i) {
                    return !eventPoint.has_value() || LexicographicalLess<Vertex2D>::Compare(i, eventPoint.value());
                })
                .value_or(false);
        }

        struct Initializer {
        public:
            using edges_type = std::array<std::optional<ConvexPolygon::Edge>, 4>;

            Initializer(const edges_type& edges, const std::optional<Vertex2D>& top, ConvexPolygon& output)
                : Edges{CreateEdges(edges, top)}
                , Top(top)
                , Output(output) {}

            void Initialize() const {
                const EdgeLeftToRightComparator comparator{Top};
                if (Edges.at(2).has_value() && Edges.at(1).has_value()
                    && comparator(Edges.at(1)->BoundaryEdge, Edges.at(2)->BoundaryEdge)) {
                    // 2: l1 r1 l2 r2
                    if (Edges.at(1).has_value()) {
                        HandleRightEdge(Edges.at(1).value(), Edges.at(2), Edges.at(3));
                    }
                    if (Edges.at(2).has_value()) {
                        HandleLeftEdge(Edges.at(2).value(), Edges.at(0), Edges.at(1));
                    }

                } else if (
                    (Edges.at(1).has_value() && Edges.at(3).has_value()
                     && comparator(Edges.at(1)->BoundaryEdge, Edges.at(3)->BoundaryEdge))
                    || (Edges.at(1).has_value() && !Edges.at(3).has_value())
                ) {
                    // 3: l1 l2 r1 r2
                    if (Edges.at(2).has_value()) {
                        HandleLeftEdge(Edges.at(2).value(), Edges.at(0), Edges.at(1));
                    }
                    if (Edges.at(1).has_value()) {
                        HandleRightEdge(Edges.at(1).value(), Edges.at(2), Edges.at(3));
                    }
                } else {
                    // 1: l1 l2 r2 r1
                    if (Edges.at(2).has_value()) {
                        HandleLeftEdge(Edges.at(2).value(), Edges.at(0), Edges.at(1));
                    }
                    if (Edges.at(3).has_value()) {
                        HandleRightEdge(Edges.at(3).value(), Edges.at(0), Edges.at(1));
                    }
                }
            }

        private:
            void HandleRightEdge(
                const ConvexPolygon::Edge& rightEdge, const std::optional<ConvexPolygon::Edge>& otherLeftEdge,
                const std::optional<ConvexPolygon::Edge>& otherRightEdge
            ) const {
                const details::EdgeLeftToRightComparator comparator{Top};

                if (IsBetween(rightEdge, otherLeftEdge, otherRightEdge)) {
                    Output.AddRightBoundary(rightEdge.GetBoundaryHalfPlane());
                }
                if (IsLeftOfAndIntersects(rightEdge, otherLeftEdge)) {
                    Output.AddRightBoundary(rightEdge.GetBoundaryHalfPlane());
                }
                if (IntersectsBelow(rightEdge, otherRightEdge, Top)) {
                    if (comparator(otherRightEdge->BoundaryEdge, rightEdge.BoundaryEdge)) {
                        Output.AddRightBoundary(otherRightEdge.value().GetBoundaryHalfPlane());
                        Output.AddRightBoundary(rightEdge.GetBoundaryHalfPlane());
                    } else {
                        Output.AddRightBoundary(otherRightEdge.value().GetBoundaryHalfPlane());
                    }
                }
            }

            void HandleLeftEdge(
                const ConvexPolygon::Edge& leftEdge, const std::optional<ConvexPolygon::Edge>& otherLeftEdge,
                const std::optional<ConvexPolygon::Edge>& otherRightEdge
            ) const {
                const details::EdgeLeftToRightComparator comparator{Top};

                if (IsBetween(leftEdge, otherLeftEdge, otherRightEdge)) {
                    Output.AddLeftBoundary(leftEdge.GetBoundaryHalfPlane());
                }
                if (IsRightOfAndIntersectsBelow(leftEdge, otherRightEdge)) {
                    Output.AddLeftBoundary(leftEdge.GetBoundaryHalfPlane());
                }
                if (IntersectsBelow(leftEdge, otherLeftEdge, Top)) {
                    if (comparator(leftEdge.BoundaryEdge, otherLeftEdge->BoundaryEdge)) {
                        Output.AddLeftBoundary(otherLeftEdge.value().GetBoundaryHalfPlane());
                        Output.AddLeftBoundary(leftEdge.GetBoundaryHalfPlane());
                    } else {
                        Output.AddLeftBoundary(otherLeftEdge.value().GetBoundaryHalfPlane());
                    }
                }
            }

            [[nodiscard]] bool IsBetween(
                const ConvexPolygon::Edge& edge, const std::optional<ConvexPolygon::Edge>& otherLeftEdge,
                const std::optional<ConvexPolygon::Edge>& otherRightEdge
            ) const {
                const details::EdgeLeftToRightComparator comparator{Top};
                const auto rightOfLeft = otherLeftEdge
                                             .transform([&comparator, &edge](const auto& leftEdge) {
                                                 return comparator(leftEdge.BoundaryEdge, edge.BoundaryEdge);
                                             })
                                             .value_or(true);

                const auto leftOfRight = otherRightEdge
                                             .transform([&comparator, &edge](const auto& rightEdge) {
                                                 return comparator(edge.BoundaryEdge, rightEdge.BoundaryEdge);
                                             })
                                             .value_or(true);

                return rightOfLeft && leftOfRight;
            }

            [[nodiscard]] bool IsLeftOfAndIntersects(
                const ConvexPolygon::Edge& rightEdge, const std::optional<ConvexPolygon::Edge>& otherLeftEdge
            ) const {
                if (!otherLeftEdge.has_value()) {
                    return false;
                }
                const auto& leftEdge = otherLeftEdge.value();
                const details::EdgeLeftToRightComparator comparator{Top};
                const auto leftOfLeft = comparator(rightEdge.BoundaryEdge, leftEdge.BoundaryEdge);
                const bool intersectsBelow
                    = FindEdgeIntersection(leftEdge.BoundaryEdge, rightEdge.BoundaryEdge)
                          .transform([this](const Vertex2D& i) {
                              return !Top.has_value() || LexicographicalLess<Vertex2D>::Compare(i, Top.value());
                          })
                          .value_or(false);
                return leftOfLeft && intersectsBelow;
            }

            [[nodiscard]] bool IsRightOfAndIntersectsBelow(
                const ConvexPolygon::Edge& leftEdge, const std::optional<ConvexPolygon::Edge>& otherRightEdge
            ) const {
                if (!otherRightEdge.has_value()) {
                    return false;
                }

                const auto& rightEdge = otherRightEdge.value();
                const details::EdgeLeftToRightComparator comparator{Top};
                const auto rightOfRight = comparator(rightEdge.BoundaryEdge, leftEdge.BoundaryEdge);
                const bool intersectsBelow
                    = FindEdgeIntersection(leftEdge.BoundaryEdge, rightEdge.BoundaryEdge)
                          .transform([this](const Vertex2D& i) {
                              return !Top.has_value() || LexicographicalLess<Vertex2D>::Compare(i, Top.value());
                          })
                          .value_or(false);
                return rightOfRight && intersectsBelow;
            }

            static edges_type CreateEdges(const edges_type& edges, const std::optional<Vertex2D>& top) {
                const EdgeLeftToRightComparator comparator{top};
                if ((edges.at(0).has_value() && edges.at(2).has_value()
                     && comparator(edges.at(2)->BoundaryEdge, edges.at(0)->BoundaryEdge))
                    || (edges.at(0).has_value() && !edges.at(2).has_value())) {
                    return edges_type{edges.at(2), edges.at(3), edges.at(0), edges.at(1)};
                }
                return edges;
            }

        private:
            edges_type Edges;
            std::optional<Vertex2D> Top;
            ConvexPolygon& Output;
        };

        struct EdgeHandler {
        public:
            using edges_type = std::array<std::optional<ConvexPolygon::Edge>, 4>;

            EdgeHandler(const edges_type& edges, ConvexPolygon& output)
                : Edges(edges)
                , Output(output) {}

            void HandleEdge(std::size_t edgeIndex) const {
                const auto& c1LeftEdge = Edges.at(0);
                const auto& c1RightEdge = Edges.at(1);
                const auto& c2LeftEdge = Edges.at(2);
                const auto& c2RightEdge = Edges.at(3);

                auto isLeftEdge = [](const auto i) { return i % 2 == 0; };
                auto isC1Edge = [](const auto i) { return i < 2; };

                if (isLeftEdge(edgeIndex)) {
                    if (isC1Edge(edgeIndex)) {
                        HandleLeftEdge(c1LeftEdge.value(), c2LeftEdge, c2RightEdge);
                    } else {
                        HandleLeftEdge(c2LeftEdge.value(), c1LeftEdge, c1RightEdge);
                    }
                } else {
                    if (isC1Edge(edgeIndex)) {
                        HandleRightEdge(c1RightEdge.value(), c2LeftEdge, c2RightEdge);
                    } else {
                        HandleRightEdge(c2RightEdge.value(), c1LeftEdge, c1RightEdge);
                    }
                }
            }

        private:
            void HandleRightEdge(
                const ConvexPolygon::Edge& rightEdge, const std::optional<ConvexPolygon::Edge>& otherLeftEdge,
                const std::optional<ConvexPolygon::Edge>& otherRightEdge
            ) const {
                const EdgeComparator comparator{rightEdge};

                if (IsBetween(rightEdge, otherLeftEdge, otherRightEdge)) {
                    Output.AddRightBoundary(rightEdge.GetBoundaryHalfPlane());
                }
                if (IsLeftOfAndIntersects(rightEdge, otherLeftEdge, rightEdge.UpperEndpoint)) {
                    Output.AddRightBoundary(rightEdge.GetBoundaryHalfPlane());
                    if (rightEdge.UpperEndpoint.has_value()) {
                        Output.AddLeftBoundary(otherLeftEdge.value().GetBoundaryHalfPlane());
                    }
                }
                if (IntersectsBelow(rightEdge, otherRightEdge, rightEdge.UpperEndpoint)) {
                    if (comparator.IsRightOf(otherRightEdge.value())) {
                        Output.AddRightBoundary(otherRightEdge.value().GetBoundaryHalfPlane());
                        Output.AddRightBoundary(rightEdge.GetBoundaryHalfPlane());
                    } else {
                        Output.AddRightBoundary(otherRightEdge.value().GetBoundaryHalfPlane());
                    }
                }
            }

            void HandleLeftEdge(
                const ConvexPolygon::Edge& leftEdge, const std::optional<ConvexPolygon::Edge>& otherLeftEdge,
                const std::optional<ConvexPolygon::Edge>& otherRightEdge
            ) const {
                const EdgeComparator comparator{leftEdge};

                if (IsBetween(leftEdge, otherLeftEdge, otherRightEdge)) {
                    Output.AddLeftBoundary(leftEdge.GetBoundaryHalfPlane());
                }
                if (IsRightOfAndIntersectsBelow(leftEdge, otherRightEdge, leftEdge.UpperEndpoint)) {
                    Output.AddLeftBoundary(leftEdge.GetBoundaryHalfPlane());
                    if (leftEdge.UpperEndpoint.has_value()) {
                        Output.AddRightBoundary(otherRightEdge.value().GetBoundaryHalfPlane());
                    }
                }
                if (IntersectsBelow(leftEdge, otherLeftEdge, leftEdge.UpperEndpoint)) {
                    if (comparator.IsLeftOf(otherLeftEdge.value())) {
                        Output.AddLeftBoundary(otherLeftEdge.value().GetBoundaryHalfPlane());
                        Output.AddLeftBoundary(leftEdge.GetBoundaryHalfPlane());
                    } else {
                        Output.AddLeftBoundary(otherLeftEdge.value().GetBoundaryHalfPlane());
                    }
                }
            }

            bool IsBetween(
                const ConvexPolygon::Edge& edge, const std::optional<ConvexPolygon::Edge>& otherLeftEdge,
                const std::optional<ConvexPolygon::Edge>& otherRightEdge
            ) const {
                const details::EdgeComparator comparator{edge};
                const auto rightOfLeft
                    = otherLeftEdge
                          .transform([&comparator](const auto& leftEdge) { return comparator.IsRightOf(leftEdge); })
                          .value_or(true);

                const auto leftOfRight
                    = otherRightEdge
                          .transform([&comparator](const auto& rightEdge) { return comparator.IsLeftOf(rightEdge); })
                          .value_or(true);

                return rightOfLeft && leftOfRight;
            }

            bool IsRightOfAndIntersectsBelow(
                const ConvexPolygon::Edge& leftEdge, const std::optional<ConvexPolygon::Edge>& otherRightEdge,
                const std::optional<Vertex2D>& eventPoint
            ) const {
                if (!otherRightEdge.has_value()) {
                    return false;
                }

                const auto& rightEdge = otherRightEdge.value();
                const details::EdgeComparator comparator{leftEdge};
                const auto rightOfRight = comparator.IsRightOf(rightEdge);
                const bool intersectsBelow
                    = details::FindEdgeIntersection(leftEdge.BoundaryEdge, rightEdge.BoundaryEdge)
                          .transform([&eventPoint](const Vertex2D& i) {
                              return !eventPoint.has_value()
                                     || LexicographicalLess<Vertex2D>::Compare(i, eventPoint.value());
                          })
                          .value_or(false);
                return rightOfRight && intersectsBelow;
            }

            bool IsLeftOfAndIntersects(
                const ConvexPolygon::Edge& rightEdge, const std::optional<ConvexPolygon::Edge>& otherLeftEdge,
                const std::optional<Vertex2D>& eventPoint
            ) const {
                if (!otherLeftEdge.has_value()) {
                    return false;
                }
                const auto& leftEdge = otherLeftEdge.value();
                const details::EdgeComparator comparator{rightEdge};
                const auto leftOfLeft = comparator.IsLeftOf(leftEdge);
                const bool intersectsBelow
                    = details::FindEdgeIntersection(leftEdge.BoundaryEdge, rightEdge.BoundaryEdge)
                          .transform([&eventPoint](const Vertex2D& i) {
                              return !eventPoint.has_value()
                                     || LexicographicalLess<Vertex2D>::Compare(i, eventPoint.value());
                          })
                          .value_or(false);
                return leftOfLeft && intersectsBelow;
            }

        private:
            edges_type Edges;
            ConvexPolygon& Output;
        };
    } // namespace details

    class HalfPlaneIntersector {
    public:
        using half_plane_type = HalfPlane<Hyperplane<2UZ>>;
        [[nodiscard]] ConvexPolygon Intersect(const std::vector<half_plane_type>& halfPlanes) const {
            return Intersect(halfPlanes.begin(), halfPlanes.end());
        }

        ConvexPolygon Intersect(const ConvexPolygon& c1, const ConvexPolygon& c2) const {
            const auto top1 = FindTopVertex(c1);
            const auto top2 = FindTopVertex(c2);
            const auto top = std::invoke([&top1, &top2]() -> std::optional<Vertex2D> {
                if (top1.has_value() && top2.has_value()) {
                    return VertexBelowComparator::Compare(top1.value(), top2.value()) ? top1.value() : top2.value();
                }
                return top1.has_value() ? top1 : top2;
            });

            if (details::IsTopBelowPolygon(top, c1) || details::IsTopBelowPolygon(top, c2)) {
                return {};
            }

            std::array edges{
                FindLeftIntersectionEdge(c1, top), FindRightIntersectionEdge(c1, top),
                FindLeftIntersectionEdge(c2, top), FindRightIntersectionEdge(c2, top)
            };

            auto hasNextEdge = [](const auto& maybeEdge) {
                return maybeEdge.has_value() && maybeEdge->LowerEndpoint.has_value()
                       && maybeEdge->BoundaryIndex + 1 < maybeEdge->GetBoundary().size();
            };
            ConvexPolygon result;
            details::Initializer initializer{edges, top, result};
            initializer.Initialize();

            auto edgeToHandle
                = MaximumWithProperty(edges, hasNextEdge, VertexBelowComparator{}, [](const auto& maybeEdge) {
                      return maybeEdge->LowerEndpoint.value();
                  });
            while (edgeToHandle.has_value()) {
                const auto edgeIndex = std::distance(edges.cbegin(), edgeToHandle.value());
                COMPG_ASSERT(edges.at(edgeIndex).has_value(), "Oops");
                GoToNextEdge(edges.at(edgeIndex).value());
                details::EdgeHandler edgeHandler{edges, result};
                edgeHandler.HandleEdge(edgeIndex);

                edgeToHandle
                    = MaximumWithProperty(edges, hasNextEdge, VertexBelowComparator{}, [](const auto& maybeEdge) {
                          return maybeEdge->LowerEndpoint.value();
                      });
            }
            return result;
        }

    private:
        [[nodiscard]] ConvexPolygon Intersect(auto begin, auto end) const {
            COMPG_ASSERT(begin != end, "Expected at least one half plane");
            if (std::next(begin) == end) {
                const half_plane_type& halfPlane = *begin;
                return IsLeftBoundary(halfPlane)
                           ? ConvexPolygon{.LeftBoundary = {halfPlane.Plane}, .RightBoundary = {}}
                           : ConvexPolygon{.LeftBoundary = {}, .RightBoundary = {halfPlane.Plane}};
            }
            const auto middle = std::next(begin, std::distance(begin, end) / 2);
            const ConvexPolygon c1 = Intersect(begin, middle);
            const ConvexPolygon c2 = Intersect(middle, end);
            return Intersect(c1, c2);
        }

        static bool IsLeftBoundary(const half_plane_type& halfPlane) {
            const auto dot = halfPlane.Plane.GetNormal()[0];
            if (math::IsZero(dot)) {
                return halfPlane.Plane.GetNormal()[1] > 0;
            }
            return dot > 0;
        }
    };
} // namespace compg
