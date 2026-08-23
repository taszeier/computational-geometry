#pragma once

#include "data_structures/DoublyConnectedEdgeList.hpp"
#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include "intersection/BooleanOperations.hpp"
#include "intersection/OverlayCalculator.hpp"
#include "math/Conversions.hpp"
#include "math/primitives/Polygon.hpp"
#include "motion_planning/MinkowskiSumCalculator.hpp"

namespace compg {
    /**
     * @brief Find the union of a range of polygons by overlaying them.
     */
    DoublyConnectedEdgeList Union(auto begin, auto end) {
        COMPG_ASSERT(begin != end, "Expected at least one polygon");
        if (std::next(begin) == end) {
            auto edgeList = ConvertTo<DoublyConnectedEdgeList>(*begin);
            UpdateFaces(edgeList);
            return edgeList;
        }

        const auto middle = std::next(begin, std::distance(begin, end) / 2);
        const auto edgeList1 = Union(begin, middle);
        const auto edgeList2 = Union(middle, end);

        const OverlayCalculator calculator;
        return calculator.FindOverlay(edgeList1, edgeList2);
    }

    /**
     * @brief Convert an outer boundary in a doubly connected edge list to a polygon.
     * @param edgeList A doubly connected edge list.
     * @param boundaryEdge An edge index on an outer boundary.
     * @return The polygon representing the boundary.
     */
    Polygon
    ConvertOuterBoundary(const DoublyConnectedEdgeList& edgeList, DoublyConnectedEdgeList::edge_index boundaryEdge);

    /**
     * @brief Convert the inner boundary on the unbounded face to a polygon. The unbounded face must have exactly one
     * inner boundary.
     * @param edgeList A doubly connected edge list.
     * @return A polygon representing the boundary.
     */
    Polygon ConvertOuterBoundary(const DoublyConnectedEdgeList& edgeList);

    /**
     * @brief Convert all boundaries on the unbounded face of the edge list to polygons.
     * @param edgeList A doubly connected edge list.
     * @return A vector of polygons, one for each boundary.
     */
    std::vector<Polygon> ConvertOuterBoundaries(const DoublyConnectedEdgeList& edgeList);

    inline bool IsBoxValid(const Box2D& box) {
        return box.GetSideLength(0) > 0 && box.GetSideLength(1) > 0;
    }

    std::vector<LineSegment2D> GetEdges(const std::vector<Polygon>& polygons);
    std::optional<Box2D> ShrinkBox(const Box2D& box, const Polygon& polygon);
    std::vector<Polygon> FindForbiddenSpace(const Box2D& box, const std::vector<Polygon>& expandedObstacles);

    template <typename PointPathCalculatorType>
    std::optional<PointPathCalculatorType> CreatePathCalculator(
        const Polygon& convexPolygon, const Box2D& box, const std::vector<Polygon>& obstacles, std::size_t seed
    ) {
        COMPG_ASSERT(IsBoxValid(box), "Expected a valid box");
        const auto polygonMirrored = Polygon{
            convexPolygon.GetVertices() | std::views::transform([](const auto& v) { return Vertex2D{-v}; })
            | std::ranges::to<std::vector>()
        };

        return ShrinkBox(box, polygonMirrored)
            .transform([&polygonMirrored, &obstacles, seed](const Box2D& shrunkedBox) {
                const MinkowskiSumCalculator sumCalculator;
                const auto expandedObstacles
                    = obstacles | std::views::transform([&sumCalculator, &polygonMirrored](const auto& p) {
                          return sumCalculator.FindHalfConvexSum(p, polygonMirrored);
                      })
                      | std::ranges::to<std::vector>();
                const auto forbiddenSpace = FindForbiddenSpace(shrunkedBox, expandedObstacles);
                return PointPathCalculatorType{shrunkedBox, forbiddenSpace, seed};
            });
    }
} // namespace compg