#pragma once

#include "common/Random.hpp"
#include "math/primitives/Polygon.hpp"
#include "motion_planning/FreeSpace.hpp"

namespace compg {
    class FreeSpaceCalculator {
    public:
        /**
         * @param box The box that bounds the free space. All obstacles must be inside the box.
         * @param obstacles A vector of polygons representing the obstacles.
         * @param seed The seed used to improve average running time.
         * @return The free space.
         */
        [[nodiscard]] FreeSpace
        FindFreeSpace(const Box2D& box, const std::vector<Polygon>& obstacles, std::size_t seed = DEFAULT_SEED) const;
    };
} // namespace compg
