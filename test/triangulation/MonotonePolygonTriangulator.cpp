#include <catch2/catch_test_macros.hpp>

#include "triangulation/MonotonePolygonTriangulator.hpp"

#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"

using namespace compg;

TEST_CASE("Monotone polygon with horizontal edge is triangulated", "[MonotonePolygonTriangulator]") {
    DoublyConnectedEdgeList polygon;
    const auto v0 = polygon.InsertVertex({1, 1});
    const auto v1 = polygon.InsertVertex({3, 0});
    const auto v2 = polygon.InsertVertex({4, 2});
    const auto v3 = polygon.InsertVertex({2, 2});
    const auto v4 = polygon.InsertVertex({1, 3});
    const auto v5 = polygon.InsertVertex({0, 2});

    const auto e0 = polygon.InsertEdge(v0, v1);
    polygon.InsertEdge(v1, v2);
    polygon.InsertEdge(v2, v3);
    polygon.InsertEdge(v3, v4);
    polygon.InsertEdge(v4, v5);
    polygon.InsertEdge(v5, v0);

    DoublyConnectedEdgeList expected{polygon};
    expected.InsertEdge(v3, v5);
    expected.InsertEdge(v0, v2);
    expected.InsertEdge(v0, v3);

    MonotonePolygonTriangulator triangulator{};
    triangulator.Triangulate(polygon, polygon.GetTwinIndex(e0));

    REQUIRE(AreIsomorphic(expected, polygon));
}