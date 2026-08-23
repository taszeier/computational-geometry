#include "motion_planning/FreeSpaceCalculator.hpp"

#include "data_structures/AvlTree.hpp"
#include "math/BoundingBox.hpp"
#include "math/Geometry.hpp"
#include "motion_planning/ObstacleManipulation.hpp"

namespace compg {
    namespace details {
        void FindSideEdges(
            const LineSegment2D& side, const std::vector<LineSegment2D>& polygonEdges, auto projector,
            std::vector<LineSegment2D>& edges, std::unordered_set<std::size_t>& collinearEdges
        ) {

            const Less less{};
            AvlTree<Vertex2D> vertices;
            vertices.Insert(side[0], less, projector);
            vertices.Insert(side[1], less, projector);

            for (const auto& [index, edge] : polygonEdges | std::views::enumerate) {
                const auto side0 = FindSide(side, edge[0]);
                const auto side1 = FindSide(side, edge[1]);

                if (side0 == PointSide::Collinear) {
                    vertices.Insert(edge[0], less, projector);
                }
                if (side1 == PointSide::Collinear) {
                    vertices.Insert(edge[1], less, projector);
                }

                if (side0 == PointSide::Collinear && side1 == PointSide::Collinear) {
                    collinearEdges.insert(index);
                }
            }

            auto prev = vertices.begin();
            auto current = std::next(prev);

            while (current != vertices.end()) {
                edges.emplace_back(*prev, *current);

                prev = current;
                ++current;
            }
        }

        auto CreateBoxEdges(const Box2D& box, const std::vector<LineSegment2D>& polygonEdges) {
            std::vector<LineSegment2D> edges;
            std::unordered_set<std::size_t> collinearEdges;

            const auto xProj = CoordinateProjector<2>(0);
            const auto yProj = CoordinateProjector<2>(1);

            const auto [v0, v1, v2, v3] = GetCorners(box);

            FindSideEdges(LineSegment2D{v3, v2}, polygonEdges, xProj, edges, collinearEdges);
            FindSideEdges(LineSegment2D{v0, v1}, polygonEdges, xProj, edges, collinearEdges);
            FindSideEdges(LineSegment2D{v0, v3}, polygonEdges, yProj, edges, collinearEdges);
            FindSideEdges(LineSegment2D{v1, v2}, polygonEdges, yProj, edges, collinearEdges);

            return std::tuple{edges, collinearEdges};
        }
    } // namespace details

    FreeSpace
    FreeSpaceCalculator::FindFreeSpace(const Box2D& box, const std::vector<Polygon>& polygons, std::size_t seed) const {
        const auto polygonEdges = GetEdges(polygons);
        const auto edgesBox = FindBoundingBox(polygonEdges);
        COMPG_ASSERT(IsBoxValid(box), "Expected a valid box");
        COMPG_ASSERT(
            box.Contains(edgesBox.GetLowerCorner()) && box.Contains(edgesBox.GetUpperCorner()),
            "The box does not contain the obstacles."
        );

        const auto [boxEdges, collinearEdges] = details::CreateBoxEdges(box, polygonEdges);

        TrapezoidalMap map{Pad(box, 1.0, 1.0)};
        TrapezoidalSearchStructure searchStructure{0UZ};

        const auto permutation = RandomIndexPermutation(polygonEdges.size() + boxEdges.size(), seed);

        std::ranges::for_each(permutation, [&polygonEdges, &map, &searchStructure, &boxEdges, &collinearEdges](auto i) {
            if (collinearEdges.contains(i)) {
                return;
            }

            const auto& segment = i < polygonEdges.size() ? polygonEdges.at(i) : boxEdges.at(i - polygonEdges.size());
            const auto intersectingTrapezoids = searchStructure.FollowSegment(map, segment);
            const auto splitResult = map.Split(segment, intersectingTrapezoids);
            searchStructure.UpdateAfterSplit(intersectingTrapezoids, splitResult, segment);
        });

        const std::unordered_set<LineSegment2D> edges{polygonEdges.begin(), polygonEdges.end()};
        decltype(FreeSpace::ObstacleTrapezoids) obstacleTrapezoids;
        std::ranges::for_each(
            std::views::iota(0UZ, map.NumTrapezoids()), [&map, &edges, &obstacleTrapezoids](const auto trapezoidIndex) {
                const auto bottomSegment = map.GetTrapezoid(trapezoidIndex).BottomSegment;
                if (edges.contains(bottomSegment)
                    && LexicographicalLess<Vertex2D>::Compare(bottomSegment[0], bottomSegment[1])) {
                    obstacleTrapezoids.insert(trapezoidIndex);
                }
            }
        );

        return FreeSpace{box, std::move(map), std::move(searchStructure), std::move(obstacleTrapezoids)};
    }
} // namespace compg