#pragma once
#include "point_location/TrapezoidalMap.hpp"
#include "point_location/TrapezoidalSearchStructure.hpp"

namespace compg {
    /**
     * @brief Represents the regions of the plane where the robot does not intersect any obstacles.
     */
    struct FreeSpace {
        Box2D Box;
        TrapezoidalMap Map;
        TrapezoidalSearchStructure SearchStructure;
        std::unordered_set<TrapezoidalMap::trapezoid_index> ObstacleTrapezoids;
    };

    bool Contains(const FreeSpace& freeSpace, const Vertex2D& vertex);
} // namespace compg
