#include "motion_planning/ConvexPathCalculator.hpp"

#include "motion_planning/ObstacleManipulation.hpp"
#include "motion_planning/PointPathCalculator.hpp"

namespace compg {
    ConvexPathCalculator::ConvexPathCalculator(
        const Polygon& convexPolygon, const Box2D& box, const std::vector<Polygon>& obstacles, std::size_t seed
    )
        : PathCalculator{CreatePathCalculator<PointPathCalculator>(convexPolygon, box, obstacles, seed)} {}

    std::optional<std::vector<Vertex2D>>
    ConvexPathCalculator::FindPath(const Vertex2D& start, const Vertex2D& goal) const {
        return PathCalculator.and_then([&start, &goal](const auto& c) { return c.FindPath(start, goal); });
    }
} // namespace compg