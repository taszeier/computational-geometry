#pragma once

#include "common/Random.hpp"
#include "common/Vertex.hpp"
#include "convex_hull/ConvexHull.hpp"

namespace compg {
    class ConvexHull3DCalculator {
    public:
        explicit ConvexHull3DCalculator(Float epsilon = EPSILON, std::size_t seed = DEFAULT_SEED)
            : Epsilon(epsilon)
            , Seed(seed) {}

        /**
         * @brief Find the convex hull of 3D vertices.
         * @param vertices A vector of non-coplanar 3D vertices.
         * @return The convex hull of the vertices.
         */
        [[nodiscard]] ConvexHull3D FindConvexHull(const std::vector<Vertex3D>& vertices) const;

    private:
        Float Epsilon;
        std::size_t Seed;
    };
} // namespace compg