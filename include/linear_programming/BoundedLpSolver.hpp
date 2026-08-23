#pragma once

#include "common/Random.hpp"
#include "common/Vertex.hpp"
#include "linear_programming/SolutionTypes.hpp"
#include "math/primitives/HalfPlane.hpp"
#include "math/primitives/Hyperplane.hpp"

namespace compg {

    class BoundedLpSolver {
    public:
        using half_plane_type = HalfPlane<Hyperplane<2UZ>>;
        using solution_type = std::variant<Infeasible, Bounded>;

        [[nodiscard]] solution_type Solve(
            const std::vector<half_plane_type>& halfPlanes, Float bound, const Vertex2D& cost,
            std::size_t seed = DEFAULT_SEED
        ) const;
    };
} // namespace compg
