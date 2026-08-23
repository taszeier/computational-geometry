#include "linear_programming/BoundedLpSolver.hpp"
#include "linear_programming/LpSolver.hpp"

namespace compg {
    namespace details {
        using half_plane_type = BoundedLpSolver::half_plane_type;

        auto CreateXYBoundHalfPlanes(Float bound, const Vertex2D& cost) {
            const Float xFlip = (cost[0] <= 0) ? -1 : 1;
            const Hyperplane<2UZ> xBound{{bound * xFlip, 0}, {-1 * xFlip, 0}};
            const Float yFlip = (cost[1] <= 0) ? -1 : 1;
            const Hyperplane<2UZ> yBound{{0, bound * yFlip}, {0, -1 * yFlip}};

            return std::tuple{half_plane_type{xBound}, half_plane_type{yBound}};
        }
    } // namespace details

    BoundedLpSolver::solution_type BoundedLpSolver::Solve(
        const std::vector<half_plane_type>& halfPlanes, Float bound, const Vertex2D& cost, std::size_t seed
    ) const {
        const auto [xBound, yBound] = details::CreateXYBoundHalfPlanes(bound, cost);
        auto permutation{halfPlanes};
        RandomPermutation(permutation, seed);
        const LpSolver solver{};
        return solver.Solve(permutation, cost, xBound, yBound);
    }
} // namespace compg