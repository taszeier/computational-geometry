#include <catch2/catch_test_macros.hpp>

#include "triangulation/PolygonTriangulator.hpp"

#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"

using namespace compg;

TEST_CASE("Polygon with all five types of vertices gets triangulated", "[PolygonTriangulator]") {
    DoublyConnectedEdgeList polygon;
    const auto v0 = polygon.InsertVertex({-3, -1});
    const auto v1 = polygon.InsertVertex({-2, -2});
    const auto v2 = polygon.InsertVertex({0, 0});
    const auto v3 = polygon.InsertVertex({3, -3});
    const auto v4 = polygon.InsertVertex({6, 0});
    const auto v5 = polygon.InsertVertex({3, 3});
    const auto v6 = polygon.InsertVertex({2, 2});
    const auto v7 = polygon.InsertVertex({1, 3});
    const auto v8 = polygon.InsertVertex({-1, 1});
    const auto v9 = polygon.InsertVertex({-2, 2});

    polygon.InsertEdge(v0, v1);
    polygon.InsertEdge(v1, v2);
    polygon.InsertEdge(v2, v3);
    polygon.InsertEdge(v3, v4);
    polygon.InsertEdge(v4, v5);
    polygon.InsertEdge(v5, v6);
    polygon.InsertEdge(v6, v7);
    polygon.InsertEdge(v7, v8);
    polygon.InsertEdge(v8, v9);
    polygon.InsertEdge(v9, v0);

    DoublyConnectedEdgeList expected{polygon};
    expected.InsertEdge(v2, v8);
    expected.InsertEdge(v0, v2);
    expected.InsertEdge(v0, v8);
    expected.InsertEdge(v8, v6);
    expected.InsertEdge(v2, v6);
    expected.InsertEdge(v2, v4);
    expected.InsertEdge(v6, v4);

    PolygonTriangulator triangulator{};
    triangulator.Triangulate(polygon);

    REQUIRE(AreIsomorphic(expected, polygon));
}