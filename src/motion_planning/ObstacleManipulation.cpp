#include "motion_planning/ObstacleManipulation.hpp"

namespace compg {
    Polygon
    ConvertOuterBoundary(const DoublyConnectedEdgeList& edgeList, DoublyConnectedEdgeList::edge_index boundaryEdge) {
        std::vector<Vertex2D> vertices;
        WalkBoundary(edgeList, boundaryEdge, [&edgeList, &vertices](const auto edgeIndex) {
            const auto originIndex = edgeList.GetOriginIndex(edgeIndex);
            vertices.push_back(edgeList.GetVertex(originIndex).Vertex);
        });

        std::ranges::reverse(vertices);
        return Polygon{vertices};
    }

    Polygon ConvertOuterBoundary(const DoublyConnectedEdgeList& edgeList) {
        const auto unboundedFace = edgeList.FindUnboundedFaceIndex();
        COMPG_ASSERT(edgeList.GetFace(unboundedFace).InnerComponents.size() == 1, "Oops");

        const auto boundaryEdge = edgeList.GetFace(unboundedFace).InnerComponents.at(0);
        return ConvertOuterBoundary(edgeList, boundaryEdge);
    }

    std::vector<Polygon> ConvertOuterBoundaries(const DoublyConnectedEdgeList& edgeList) {
        const auto unboundedFace = edgeList.FindUnboundedFaceIndex();
        const auto innerComponents = edgeList.GetFace(unboundedFace).InnerComponents;

        std::vector<Polygon> result;
        result.reserve(innerComponents.size());
        for (const auto boundaryIndex : innerComponents) {
            result.push_back(ConvertOuterBoundary(edgeList, boundaryIndex));
        }

        return result;
    }

    std::vector<LineSegment2D> GetEdges(const std::vector<Polygon>& polygons) {
        std::vector<LineSegment2D> edges;
        const auto numEdges = std::ranges::fold_left(
            polygons | std::views::transform([](const auto& p) { return p.NumEdges(); }), 0UZ, std::plus{}
        );
        edges.reserve(numEdges);

        std::ranges::for_each(polygons, [&edges](const Polygon& polygon) {
            std::ranges::for_each(std::views::iota(0UZ, polygon.NumEdges()), [&polygon, &edges](const auto i) {
                edges.emplace_back(polygon.GetEdge(i));
            });
        });

        return edges;
    }

    std::optional<Box2D> ShrinkBox(const Box2D& box, const Polygon& polygon) {
        const auto& v0 = polygon.GetVertex(0);
        Float xMin = v0[0];
        Float xMax = v0[0];
        Float yMin = v0[1];
        Float yMax = v0[1];

        for (const auto& vertex : polygon.GetVertices()) {
            xMin = std::min(xMin, vertex[0]);
            xMax = std::max(xMax, vertex[0]);
            yMin = std::min(yMin, vertex[1]);
            yMax = std::max(yMax, vertex[1]);
        }

        COMPG_ASSERT(xMin <= 0 && 0 <= xMax, "Expected the the robot to contain the origin");
        COMPG_ASSERT(yMin <= 0 && 0 <= yMax, "Expected the the robot to contain the origin");

        const auto lowerCorner = box.GetLowerCorner();
        const auto upperCorner = box.GetUpperCorner();

        const auto xLower = lowerCorner[0] + xMax;
        const auto xUpper = upperCorner[0] + xMin;
        const auto yLower = lowerCorner[1] + yMax;
        const auto yUpper = upperCorner[1] + yMin;

        if (xLower < xUpper && yLower < yUpper) {
            return Box2D{{xLower, yLower}, {xUpper, yUpper}};
        }
        return std::nullopt;
    }

    std::vector<Polygon> FindForbiddenSpace(const Box2D& box, const std::vector<Polygon>& expandedObstacles) {
        // TODO: This can create invalid polygons when the intersection of two polygons is just a point
        /* TODO: Overlapping polygons can split the free space into disjoint regions that are bounded by the polygon.
           These areas are considered to be inside the polygon but should not be.
           e.g., when two unioned polygons become the area between two concentric circles. */
        const auto polygonUnion = Union(expandedObstacles.begin(), expandedObstacles.end());
        auto boxEdgeList = ConvertTo<DoublyConnectedEdgeList>(Polygon{GetCorners(box)});
        UpdateFaces(boxEdgeList);
        const auto croppedUnion = Intersection(polygonUnion, boxEdgeList);
        return ConvertOuterBoundaries(croppedUnion);
    }
} // namespace compg