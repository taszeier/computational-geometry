#include "visibility_graph/ShortestConvexPathCalculator.hpp"
#include "motion_planning/ObstacleManipulation.hpp"

namespace compg {
    ShortestConvexPathCalculator::ShortestConvexPathCalculator(
        const Polygon& convexPolygon, const Box2D& box, const std::vector<Polygon>& obstacles, std::size_t seed
    )
        : Calculator{CreatePathCalculator<ShortestPointPathCalculator>(convexPolygon, box, obstacles, seed)} {}

    std::optional<std::vector<Vertex2D>>
    ShortestConvexPathCalculator::FindPath(const Vertex2D& start, const Vertex2D& goal) const {
        return Calculator.and_then([&start, &goal](const auto& calc) { return calc.FindPath(start, goal); });
    }
} // namespace compg
