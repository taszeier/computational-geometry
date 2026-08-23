#include "motion_planning/PointPathCalculator.hpp"
#include "math/Geometry.hpp"
#include "motion_planning/FreeSpaceCalculator.hpp"

#include <queue>

#include "data_structures/graph/GraphAlgorithms.hpp"

namespace compg {
    namespace details {
        Vertex2D FindTrapezoidCenter(const TrapezoidalMap::TrapezoidRecord& trapezoid) {
            const auto x = (trapezoid.LeftPoint[0] + trapezoid.RightPoint[0]) * 0.5;
            const auto line = MakeLine2DFromX(x);
            const auto topIntersection = FindIntersection(trapezoid.TopSegment, line).value_or(trapezoid.RightPoint);
            const auto bottomIntersection
                = FindIntersection(trapezoid.BottomSegment, line).value_or(trapezoid.LeftPoint);

            return (topIntersection + bottomIntersection) * 0.5;
        }

        auto FindPointAndSegment(const TrapezoidalMap::TrapezoidRecord& t1, TrapezoidalMap::trapezoid_index t2) {
            if (t1.UpperLeft == t2) {
                return std::tuple{t1.LeftPoint, t1.TopSegment};
            }
            if (t1.LowerLeft == t2) {
                return std::tuple{t1.LeftPoint, t1.BottomSegment};
            }
            if (t1.UpperRight == t2) {
                return std::tuple{t1.RightPoint, t1.TopSegment};
            }
            if (t1.LowerRight == t2) {
                return std::tuple{t1.RightPoint, t1.BottomSegment};
            }
            COMPG_THROW("Expected the trapezoids to be adjacent");
        }

        Vertex2D
        FindVerticalExtensionCenter(const TrapezoidalMap::TrapezoidRecord& t1, TrapezoidalMap::trapezoid_index t2) {
            const auto [point, segment] = FindPointAndSegment(t1, t2);
            const auto line = MakeLine2DFromX(point[0]);
            const auto maybeIntersection = FindIntersection(segment, line);
            COMPG_ASSERT(maybeIntersection, "Expected the line to intersect the segment");

            return point * 0.5 + maybeIntersection.value() * 0.5;
        }

        std::vector<Vertex2D> CreatePointPath(
            const FreeSpace& freeSpace, const Vertex2D& start, const Vertex2D& goal,
            const std::vector<TrapezoidalMap::trapezoid_index>& trapezoids
        ) {
            std::vector<Vertex2D> points;
            points.reserve(2 * trapezoids.size() + 1);
            points.push_back(start);
            points.push_back(FindTrapezoidCenter(freeSpace.Map.GetTrapezoid(trapezoids.at(0))));

            std::ranges::for_each(trapezoids | std::views::slide(2), [&freeSpace, &points](const auto& iterable) {
                const auto trapezoidIndex1 = *iterable.begin();
                const auto& trapezoid1 = freeSpace.Map.GetTrapezoid(trapezoidIndex1);
                const auto trapezoidIndex2 = *std::next(iterable.begin());
                const auto& trapezoid2 = freeSpace.Map.GetTrapezoid(trapezoidIndex2);

                points.push_back(FindVerticalExtensionCenter(trapezoid1, trapezoidIndex2));
                points.push_back(FindTrapezoidCenter(trapezoid2));
            });

            points.push_back(goal);

            return points;
        }
    } // namespace details

    PointPathCalculator::PointPathCalculator(const Box2D& box, const std::vector<Polygon>& obstacles, std::size_t seed)
        : FreeSpace(FreeSpaceCalculator{}.FindFreeSpace(box, obstacles, seed)) {}

    std::optional<std::vector<Vertex2D>>
    PointPathCalculator::FindPath(const Vertex2D& start, const Vertex2D& goal) const {
        if (!FreeSpace.Box.Contains(start) || !FreeSpace.Box.Contains(goal)) {
            return std::nullopt;
        }

        const auto startSearchResult = FreeSpace.SearchStructure.Search(start);
        if (startSearchResult.State.index() != 0) {
            return std::nullopt;
        }
        const auto goalSearchResult = FreeSpace.SearchStructure.Search(goal);
        if (goalSearchResult.State.index() != 0) {
            return std::nullopt;
        }

        const auto startTrapezoidIndex = std::get<0>(startSearchResult.State);
        const auto goalTrapezoidIndex = std::get<0>(goalSearchResult.State);

        if (FreeSpace.ObstacleTrapezoids.contains(startTrapezoidIndex)
            || FreeSpace.ObstacleTrapezoids.contains(goalTrapezoidIndex)) {
            return std::nullopt;
        }

        if (startTrapezoidIndex == goalTrapezoidIndex) {
            return {{start, goal}};
        }

        std::unordered_set<TrapezoidalMap::trapezoid_index> visited;
        std::queue<TrapezoidalMap::trapezoid_index> queue;
        queue.push(startTrapezoidIndex);
        std::unordered_map<TrapezoidalMap::trapezoid_index, TrapezoidalMap::trapezoid_index> predecessor;

        auto insert = [&visited, &queue, &predecessor](auto prevIndex, const auto& maybeIndex) {
            if (maybeIndex && !visited.contains(maybeIndex.value())) {
                queue.push(maybeIndex.value());
                predecessor[maybeIndex.value()] = prevIndex;
            }
        };

        while (!queue.empty()) {
            const auto trapezoidIndex = queue.front();
            queue.pop();

            if (trapezoidIndex == goalTrapezoidIndex) {
                const auto path = BacktrackPath(startTrapezoidIndex, goalTrapezoidIndex, predecessor);
                return details::CreatePointPath(FreeSpace, start, goal, path);
            }
            visited.insert(trapezoidIndex);
            const auto trapezoid = FreeSpace.Map.GetTrapezoid(trapezoidIndex);
            insert(trapezoidIndex, trapezoid.UpperLeft);
            insert(trapezoidIndex, trapezoid.LowerLeft);
            insert(trapezoidIndex, trapezoid.UpperRight);
            insert(trapezoidIndex, trapezoid.LowerRight);
        }

        return std::nullopt;
    }
} // namespace compg