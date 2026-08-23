#include "linear_programming/LpSolver.hpp"
#include "math/Conversions.hpp"
#include "math/Geometry.hpp"

namespace compg {

    namespace details {
        bool HandleIntersection(
            const HalfPlane<Hyperplane<2UZ>>& addedHalfPlane, const HalfPlane<Hyperplane<2UZ>>& existingHalfPlane,
            std::optional<Vertex2D>& left, std::optional<Vertex2D>& right
        ) {
            const auto addedHalfPlaneLine = ConvertTo<Line2D>(addedHalfPlane.Plane);
            const auto existingHalfPlaneLine = ConvertTo<Line2D>(existingHalfPlane.Plane);

            const auto intersection = FindIntersection(addedHalfPlaneLine, existingHalfPlaneLine);
            if (intersection.has_value()) {
                const auto p
                    = ProjectOnto(intersection.value() + existingHalfPlane.Plane.GetNormal(), addedHalfPlaneLine);
                if (LexicographicalLess<Vertex2D>::Compare(intersection.value(), p)) {
                    if (!left.has_value()
                        || LexicographicalLess<Vertex2D>::Compare(left.value(), intersection.value())) {
                        left = intersection.value();
                    }
                } else {
                    if (!right.has_value()
                        || LexicographicalLess<Vertex2D>::Compare(intersection.value(), right.value())) {
                        right = intersection.value();
                    }
                }
            } else {
                // parallel
                // check if disjoint
                if (FindSide<2UZ>(existingHalfPlane, addedHalfPlane.Plane.GetOrigin()) == PointSide::Negative) {
                    return true;
                }
            }

            return false;
        }

        std::optional<Vertex2D> FindSolution(const auto& left, const auto& right, const Vertex2D& cost) {
            if (left.has_value() && right.has_value()) {
                if (LexicographicalLess<Vertex2D>::Compare(right.value(), left.value())) {
                    return std::nullopt;
                }
                const auto leftValue = left.value().dot(cost);
                const auto rightValue = right.value().dot(cost);
                return rightValue > leftValue ? right.value() : left.value();
            }
            COMPG_ASSERT(left.has_value() || right.has_value(), "The line is unbounded in both directions");
            return left.has_value() ? left.value() : right.value();
        }
    } // namespace details

    LpSolver::solution_type LpSolver::Solve(
        const std::vector<half_plane_type>& halfPlanes, const Vertex2D& cost, const half_plane_type& bound1,
        const half_plane_type& bound2
    ) const {
        const auto intersection = FindIntersection(bound1.Plane, bound2.Plane);
        COMPG_ASSERT(intersection.has_value(), "Oops");
        auto solution = intersection.value();

        for (const auto [halfPlaneIndex, halfPlane] : halfPlanes | std::views::enumerate) {
            if (FindSide<2UZ>(halfPlane, solution) == PointSide::Negative) {
                std::optional<Vertex2D> left;
                std::optional<Vertex2D> right;

                if (details::HandleIntersection(halfPlane, bound1, left, right)) {
                    return Infeasible{};
                }
                if (details::HandleIntersection(halfPlane, bound2, left, right)) {
                    return Infeasible{};
                }
                for (long j = 0; j < halfPlaneIndex; ++j) {
                    if (details::HandleIntersection(halfPlane, halfPlanes.at(j), left, right)) {
                        return Infeasible{};
                    }
                }

                const auto maybeSolution = details::FindSolution(left, right, cost);
                if (!maybeSolution.has_value()) {
                    return Infeasible{};
                }
                solution = maybeSolution.value();
            }
        }
        return Bounded{solution};
    }
} // namespace compg