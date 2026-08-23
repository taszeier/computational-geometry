#include "motion_planning/MinkowskiSumCalculator.hpp"
#include "motion_planning/ObstacleManipulation.hpp"

#include "data_structures/DoublyConnectedEdgeList.hpp"
#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include "intersection/OverlayCalculator.hpp"
#include "triangulation/PolygonTriangulator.hpp"

namespace compg {
    namespace details {
        Polygon
        GetFaceAsPolygon(const DoublyConnectedEdgeList& edgeList, DoublyConnectedEdgeList::face_index faceIndex) {
            const auto outerComponent = edgeList.GetFace(faceIndex).OuterComponent.value();
            std::vector<Vertex2D> vertices;
            WalkBoundary(edgeList, outerComponent, [&edgeList, &vertices](const auto edgeIndex) {
                const auto originIndex = edgeList.GetOriginIndex(edgeIndex);
                vertices.push_back(edgeList.GetVertex(originIndex).Vertex);
            });

            return Polygon{vertices};
        }
    } // namespace details

    Polygon MinkowskiSumCalculator::FindMinkowskiSum(const Polygon& p1, const Polygon& p2) const {
        auto edgeList1 = ConvertTo<DoublyConnectedEdgeList>(p1);
        auto edgeList2 = ConvertTo<DoublyConnectedEdgeList>(p2);

        const PolygonTriangulator triangulator;
        triangulator.Triangulate(edgeList1);
        UpdateFaces(edgeList1);
        triangulator.Triangulate(edgeList2);
        UpdateFaces(edgeList2);

        const auto unboundedFace1 = edgeList1.FindUnboundedFaceIndex();
        const auto unboundedFace2 = edgeList2.FindUnboundedFaceIndex();

        std::vector<Polygon> polygons;

        std::ranges::for_each(
            std::views::cartesian_product(
                std::views::iota(0UZ, edgeList1.NumFaces()), std::views::iota(0UZ, edgeList2.NumFaces())
            ),
            [this, &edgeList1, &edgeList2, unboundedFace1, unboundedFace2, &polygons](const auto& tup) {
                const auto [i, j] = tup;
                if (i != unboundedFace1 && j != unboundedFace2) {
                    const auto triangle1 = details::GetFaceAsPolygon(edgeList1, i);
                    const auto triangle2 = details::GetFaceAsPolygon(edgeList2, j);

                    auto sum = FindConvexSum(triangle1, triangle2);
                    polygons.emplace_back(std::move(sum));
                }
            }
        );

        const auto polygonUnion = Union(polygons.begin(), polygons.end());
        return ConvertOuterBoundary(polygonUnion);
    }

    Polygon MinkowskiSumCalculator::FindConvexSum(const Polygon& p1, const Polygon& p2) const {

        auto lower = [](const auto& v1, const auto& v2) { return v1[1] < v2[1] || (v1[1] == v2[1] && v1[0] < v2[0]); };
        const auto& vertices1 = p1.GetVertices();
        const auto& vertices2 = p2.GetVertices();

        const std::size_t start_i = std::distance(vertices1.begin(), std::ranges::min_element(vertices1, lower));
        const std::size_t start_j = std::distance(vertices2.begin(), std::ranges::min_element(vertices2, lower));

        auto i{start_i};
        std::size_t ii{};
        auto j{start_j};
        std::size_t jj{};

        std::vector<Vertex2D> result;
        do {
            result.push_back(vertices1.at(i) + vertices2.at(j));

            const auto next_i = (i + 1) % vertices1.size();
            const auto angle_i = Angle360({1, 0}, vertices1.at(next_i) - vertices1.at(i));

            const auto next_j = (j + 1) % vertices2.size();
            const auto angle_j = Angle360({1, 0}, vertices2.at(next_j) - vertices2.at(j));
            if (math::IsZero(angle_i - angle_j)) {
                i = next_i;
                ++ii;
                j = next_j;
                ++jj;
            } else if (angle_i < angle_j) {
                i = next_i;
                ++ii;
            } else {
                j = next_j;
                ++jj;
            }
        } while (ii != vertices1.size() && jj != vertices2.size());

        while (ii != vertices1.size()) {
            result.push_back(vertices1.at(i) + vertices2.at(j));

            const auto next_i = (i + 1) % vertices1.size();
            i = next_i;
            ++ii;
        }

        while (jj != vertices2.size()) {
            result.push_back(vertices1.at(i) + vertices2.at(j));

            const auto next_j = (j + 1) % vertices2.size();
            j = next_j;
            ++jj;
        }

        return Polygon{result};
    }

    Polygon MinkowskiSumCalculator::FindHalfConvexSum(const Polygon& polygon, const Polygon& convexPolygon) const {
        auto edgeList = ConvertTo<DoublyConnectedEdgeList>(polygon);

        const PolygonTriangulator triangulator;
        triangulator.Triangulate(edgeList);
        UpdateFaces(edgeList);

        const auto unboundedFace1 = edgeList.FindUnboundedFaceIndex();
        std::vector<Polygon> polygons;

        std::ranges::for_each(
            std::views::iota(0UZ, edgeList.NumFaces()),
            [this, &edgeList, &convexPolygon, unboundedFace1, &polygons](auto i) {
                if (i != unboundedFace1) {
                    const auto triangle = details::GetFaceAsPolygon(edgeList, i);
                    auto sum = FindConvexSum(triangle, convexPolygon);
                    polygons.emplace_back(std::move(sum));
                }
            }
        );

        const auto polygonUnion = Union(polygons.begin(), polygons.end());
        return ConvertOuterBoundary(polygonUnion);
    }
} // namespace compg
