#pragma once

#include "common/Random.hpp"
#include "common/Vertex.hpp"
#include "math/primitives/Polygon.hpp"
#include "motion_planning/FreeSpace.hpp"
#include <optional>

namespace compg {
    class PointPathCalculator {
    public:
        /**
         * @param box The box that bounds the free space. All obstacles must be inside the box.
         * @param obstacles A vector of disjoint polygons representing the obstacles.
         * @param seed The seed used to improve average running time.
         */
        PointPathCalculator(const Box2D& box, const std::vector<Polygon>& obstacles, std::size_t seed = DEFAULT_SEED);

        /**
         * @param start The start position of the robot.
         * @param goal The goal position of the robot.
         * @return A path from start to goal if it is possible to traverse the obstacles without collision, otherwise
         * std::nullopt.
         */
        std::optional<std::vector<Vertex2D>> FindPath(const Vertex2D& start, const Vertex2D& goal) const;

    private:
        FreeSpace FreeSpace;
    };
} // namespace compg
