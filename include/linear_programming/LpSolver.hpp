#pragma once

#include "linear_programming/SolutionTypes.hpp"
#include "math/primitives/HalfPlane.hpp"
#include "math/primitives/Hyperplane.hpp"

namespace compg {
    class LpSolver {
    public:
        using half_plane_type = HalfPlane<Hyperplane<2UZ>>;
        using solution_type = std::variant<Infeasible, Bounded>;

        solution_type Solve(
            const std::vector<half_plane_type>& halfPlanes, const Vertex2D& cost, const half_plane_type& bound1,
            const half_plane_type& bound2
        ) const;
    };
} // namespace compg