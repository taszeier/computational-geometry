#include <catch2/catch_test_macros.hpp>

#include "data_structures/DoublyConnectedEdgeList.hpp"

using namespace compg;

TEST_CASE("Create a DCEL representing a triangle", "[DoublyConnectedEdgeList]") {
    DoublyConnectedEdgeList edgeList;
    auto v1 = edgeList.InsertVertex({1, 1});
    auto v2 = edgeList.InsertVertex({2, 2});
    auto v3 = edgeList.InsertVertex({1, 3});

    const auto e1 = edgeList.InsertEdge(v1, v2);
    const auto e3 = edgeList.InsertEdge(v2, v3);
    const auto e5 = edgeList.InsertEdge(v1, v3);

    const auto edge1 = edgeList.GetEdge(e1);
    const auto e0 = edge1.TwinIndex;
    const auto edge0 = edgeList.GetEdge(e0);
    const auto edge3 = edgeList.GetEdge(e3);
    const auto e2 = edge3.TwinIndex;
    const auto edge2 = edgeList.GetEdge(e2);
    const auto edge5 = edgeList.GetEdge(e5);
    const auto e4 = edge5.TwinIndex;
    const auto edge4 = edgeList.GetEdge(e4);

    REQUIRE(edge1.OriginIndex == v1);
    REQUIRE(edge1.TwinIndex == e0);
    REQUIRE(edge1.NextIndex == e3);
    REQUIRE(edge1.PreviousIndex == e4);

    REQUIRE(edge0.OriginIndex == v2);
    REQUIRE(edge0.TwinIndex == e1);
    REQUIRE(edge0.NextIndex == e5);
    REQUIRE(edge0.PreviousIndex == e2);

    REQUIRE(edge3.OriginIndex == v2);
    REQUIRE(edge3.TwinIndex == e2);
    REQUIRE(edge3.NextIndex == e4);
    REQUIRE(edge3.PreviousIndex == e1);

    REQUIRE(edge2.OriginIndex == v3);
    REQUIRE(edge2.TwinIndex == e3);
    REQUIRE(edge2.NextIndex == e0);
    REQUIRE(edge2.PreviousIndex == e5);

    REQUIRE(edge5.OriginIndex == v1);
    REQUIRE(edge5.TwinIndex == e4);
    REQUIRE(edge5.NextIndex == e2);
    REQUIRE(edge5.PreviousIndex == e0);

    REQUIRE(edge4.OriginIndex == v3);
    REQUIRE(edge4.TwinIndex == e5);
    REQUIRE(edge4.NextIndex == e1);
    REQUIRE(edge4.PreviousIndex == e3);
}