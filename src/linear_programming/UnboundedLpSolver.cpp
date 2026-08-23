#include "linear_programming/UnboundedLpSolver.hpp"
#include "linear_programming/LpSolver.hpp"
#include "math/Conversions.hpp"
#include "math/Geometry.hpp"

#include <ranges>

namespace compg {

    using half_plane_type = UnboundedLpSolver::half_plane_type;

    namespace details {
        struct BoundCertificates {
            std::tuple<std::size_t, std::size_t> Certificates;
        };

        struct LeftUnbounded {
            std::size_t Ceiling;
            std::optional<std::size_t> Right;
        };

        std::variant<BoundCertificates, Unbounded, LeftUnbounded>
        FindRay(const std::vector<half_plane_type>& halfPlanes) {
            std::optional<Float> left;
            std::optional<std::size_t> leftIndex;
            std::optional<Float> right;
            std::optional<std::size_t> rightIndex;
            auto isEmpty
                = [&left, &right]() { return left.has_value() && right.has_value() && right.value() < left.value(); };
            std::optional<std::size_t> ceilingHpIndex;

            std::size_t i = 0;
            while (i < halfPlanes.size() && !isEmpty()) {
                const auto& normal = halfPlanes.at(i).Plane.GetNormal();
                if (math::IsZero(normal[0])) {
                    if (normal[1] < 0 && !ceilingHpIndex.has_value()) {
                        // normal == (0, -1)
                        ceilingHpIndex = i;
                    }
                } else {
                    const auto limit = -normal[1] / normal[0];
                    if (normal[0] > 0) {
                        if (!left.has_value() || left.value() < limit) {
                            left = limit;
                            leftIndex = i;
                        }
                    } else {
                        if (!right.has_value() || limit < right.value()) {
                            right = limit;
                            rightIndex = i;
                        }
                    }
                }
                ++i;
            }

            if (isEmpty()) {
                return BoundCertificates{{leftIndex.value(), rightIndex.value()}};
            }

            if (ceilingHpIndex.has_value()) {
                if (leftIndex.has_value()) {
                    return BoundCertificates{{leftIndex.value(), ceilingHpIndex.value()}};
                }
                return LeftUnbounded{ceilingHpIndex.value(), rightIndex};
            }

            Float dx{};
            if (left.has_value() && 0 < left.value()) {
                dx = left.value();
            } else if (right.has_value() && right.value() < 0) {
                dx = right.value();
            } else {
                dx = 0;
            }

            return Unbounded{{dx, 1}};
        }

        std::variant<Infeasible, Unbounded>
        ValidateRay(const std::vector<half_plane_type>& halfPlanes, const Vertex2D& ray) {
            std::optional<Float> left;
            std::optional<Float> right;
            auto isEmpty
                = [&left, &right]() { return left.has_value() && right.has_value() && right.value() < left.value(); };
            std::size_t i = 0;

            const auto angle = Angle180(ray, {0, 1});

            while (i < halfPlanes.size() && !isEmpty()) {
                const auto& normal = halfPlanes.at(i).Plane.GetNormal();
                if (math::IsZero(ray.dot(normal))) {

                    const auto planeRotated = Rotate(halfPlanes.at(i).Plane, angle);
                    const auto intersection = planeRotated.GetOrigin()[0];

                    if (planeRotated.GetNormal()[0] > 0) {
                        if (!left.has_value() || left.value() < intersection) {
                            left = intersection;
                        }
                    } else {
                        if (!right.has_value() || intersection < right.value()) {
                            right = intersection;
                        }
                    }
                }
                ++i;
            }
            if (isEmpty()) {
                return Infeasible{};
            }
            return Unbounded{ray};
        }

        using unbounded_test_type = std::variant<Infeasible, BoundCertificates, Unbounded>;

        unbounded_test_type PerformUnboundedTest(const std::vector<half_plane_type>& halfPlanes, const Vertex2D& cost) {
            COMPG_ASSERT(cost.squaredNorm() > EPSILON * EPSILON, "Oh no");

            const auto angle = Angle180(cost.normalized(), {0, 1});
            const auto rotatedHalfPlanes = halfPlanes
                                           | std::views::transform([angle](const half_plane_type& halfPlane) {
                                                 return half_plane_type{Rotate(halfPlane.Plane, angle)};
                                             })
                                           | std::ranges::to<std::vector>();

            const auto findRayResult = FindRay(rotatedHalfPlanes);

            auto handleUnbounded = [angle, &rotatedHalfPlanes](const Unbounded& unbounded) -> unbounded_test_type {
                const auto validateRayResult = ValidateRay(rotatedHalfPlanes, unbounded.Ray);
                return std::visit(
                    Overloads{
                        [](Infeasible r) -> unbounded_test_type { return r; },
                        [angle](const Unbounded& ub) -> unbounded_test_type {
                            return Unbounded{Rotate(ub.Ray, -angle)};
                        }
                    },
                    validateRayResult
                );
            };

            auto handleLeftUnbounded
                = [angle, &rotatedHalfPlanes, &cost](const LeftUnbounded& lub) -> unbounded_test_type {
                const auto validateRayResult = ValidateRay(rotatedHalfPlanes, {-1, 0});
                return std::visit(
                    Overloads{
                        [](Infeasible r) -> unbounded_test_type { return r; },
                        [angle, &lub, &cost](const Unbounded& ub) -> unbounded_test_type {
                            if (!lub.Right.has_value()) {
                                return Unbounded{Rotate(ub.Ray, -angle)};
                            }
                            if (Angle360(cost, {1, 0}) < math::PI) {
                                return BoundCertificates{{lub.Ceiling, lub.Right.value()}};
                            }
                            return Unbounded{Rotate(ub.Ray, -angle)};
                        }
                    },
                    validateRayResult
                );
            };

            return std::visit(
                Overloads{
                    [](const BoundCertificates& c) -> unbounded_test_type { return c; }, handleUnbounded,
                    handleLeftUnbounded
                },
                findRayResult
            );
        }

        UnboundedLpSolver::solution_type HandleBoundedCase(
            const std::vector<half_plane_type>& halfPlanes, const Vertex2D& cost, const BoundCertificates& bound,
            std::size_t seed
        ) {
            auto permutation{halfPlanes};
            auto [mi, Mi] = bound.Certificates;
            COMPG_ASSERT(mi != Mi, "Oops");
            if (mi > Mi) {
                std::swap(mi, Mi);
            }

            const auto it2 = std::next(permutation.begin(), Mi);
            const auto bound2 = *it2;
            permutation.erase(it2);

            const auto it1 = std::next(permutation.begin(), mi);
            const auto bound1 = *it1;
            permutation.erase(it1);

            RandomPermutation(permutation, seed);

            const LpSolver solver{};
            const auto solution = solver.Solve(permutation, cost, bound1, bound2);
            return std::visit([](const auto& r) -> UnboundedLpSolver::solution_type { return r; }, solution);
        }
    } // namespace details

    UnboundedLpSolver::solution_type UnboundedLpSolver::Solve(
        const std::vector<half_plane_type>& halfPlanes, const Vertex2D& cost, std::size_t seed
    ) const {
        const auto k = cost.squaredNorm() < EPSILON * EPSILON ? Vertex2D{-1, 0} : cost;
        const auto testResult = details::PerformUnboundedTest(halfPlanes, k);
        return std::visit(
            Overloads{
                [](const auto& r) -> solution_type { return r; },
                [&halfPlanes, &k, seed](const details::BoundCertificates& b) -> solution_type {
                    return details::HandleBoundedCase(halfPlanes, k, b, seed);
                }
            },
            testResult
        );
    }

} // namespace compg