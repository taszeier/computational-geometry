#pragma once

#include "math/primitives/Hyperplane.hpp"
#include "sweep_line/SweepLine.hpp"
#include <vector>

#include "math/Conversions.hpp"
#include "math/Geometry.hpp"

namespace compg {
    struct ConvexPolygon {
        using boundary_type = std::vector<Hyperplane<2UZ>>;
        boundary_type LeftBoundary;
        boundary_type RightBoundary;

        void AddLeftBoundary(const Hyperplane<2UZ>& hyperplane) {
            if (LeftBoundary.empty() || !(LeftBoundary.back() == hyperplane)) {
                LeftBoundary.push_back(hyperplane);
            }
        }

        void AddRightBoundary(const Hyperplane<2UZ>& hyperplane) {
            if (RightBoundary.empty() || !(RightBoundary.back() == hyperplane)) {
                RightBoundary.push_back(hyperplane);
            }
        }

        struct Edge {
            using boundary_edge_type = std::variant<LineSegment2D, HalfLine2D, Line2D>;
            boundary_edge_type BoundaryEdge;
            std::optional<Vertex2D> UpperEndpoint;
            std::optional<Vertex2D> LowerEndpoint;
            std::size_t BoundaryIndex;
            const ConvexPolygon& Polygon;
            bool IsLeftBoundary;

            [[nodiscard]] const auto& GetBoundaryHalfPlane() const {
                return GetBoundary().at(BoundaryIndex);
            }

            [[nodiscard]] const boundary_type& GetBoundary() const {
                return IsLeftBoundary ? Polygon.LeftBoundary : Polygon.RightBoundary;
            }

            [[nodiscard]] const boundary_type& GetOppositeBoundary() const {
                return IsLeftBoundary ? Polygon.RightBoundary : Polygon.LeftBoundary;
            }
        };
    };

    inline bool AreEqual(const ConvexPolygon& a, const ConvexPolygon& b, Float epsilon = EPSILON) {
        if (a.LeftBoundary.size() != b.LeftBoundary.size() || a.RightBoundary.size() != b.RightBoundary.size()) {
            return false;
        }
        auto boundariesAreEqual = [epsilon](const auto& b1, const auto& b2) {
            return std::ranges::all_of(std::views::zip(b1, b2), [epsilon](const auto& tup) -> bool {
                const auto& [hp1, hp2] = tup;
                return AreEqual(hp1, hp2, epsilon);
            });
        };

        return boundariesAreEqual(a.LeftBoundary, b.LeftBoundary)
               && boundariesAreEqual(a.RightBoundary, b.RightBoundary);
    }

    enum class BoundaryRelation {
        Parallel,
        BoundedAbove,
        BoundedBelow
    };

    inline BoundaryRelation
    FindBoundaryRelation(const Hyperplane<2UZ>& left, const Hyperplane<2UZ>& right, Float epsilon = EPSILON) {
        const auto angle = Angle180(right.GetNormal(), left.GetNormal());

        if (math::IsZero(angle, epsilon) || math::IsZero(angle - math::PI, epsilon)) {
            return BoundaryRelation::Parallel;
        }
        return angle < 0 ? BoundaryRelation::BoundedBelow : BoundaryRelation::BoundedAbove;
    }

    std::optional<Vertex2D> FindTopVertex(const ConvexPolygon& polygon);

    std::optional<Vertex2D> FindBottomVertex(const ConvexPolygon& polygon);

    std::optional<std::size_t> FindIntersectionBoundary(const auto& boundary, const std::optional<Vertex2D>& topVertex);

    ConvexPolygon::Edge::boundary_edge_type CreateBoundaryEdge(
        const std::optional<Vertex2D>& upperEndpoint, const std::optional<Vertex2D>& lowerEndpoint,
        const Hyperplane<2UZ>& boundary
    );

    std::optional<ConvexPolygon::Edge>
    FindLeftIntersectionEdge(const ConvexPolygon& polygon, const std::optional<Vertex2D>& vertex);

    std::optional<ConvexPolygon::Edge>
    FindRightIntersectionEdge(const ConvexPolygon& polygon, const std::optional<Vertex2D>& vertex);

    void GoToNextEdge(ConvexPolygon::Edge& edge);
} // namespace compg
