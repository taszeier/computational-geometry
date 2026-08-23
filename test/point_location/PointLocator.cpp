#include <catch2/catch_test_macros.hpp>

#include "data_structures/DoublyConnectedEdgeList.hpp"
#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include "point_location/PointLocator.hpp"

using namespace compg;

TEST_CASE("PointLocator", "[PointLocator]") {
    DoublyConnectedEdgeList edgeList;
    std::array<DoublyConnectedEdgeList::face_index, 5> faces{};
    std::array<DoublyConnectedEdgeList::edge_index, 2> edges{};
    std::array<DoublyConnectedEdgeList::vertex_index, 3> vertices{};

    {
        const auto v0 = edgeList.InsertVertex({1, 5});
        const auto v1 = edgeList.InsertVertex({2, 2});
        const auto v2 = edgeList.InsertVertex({3, 3});
        const auto v3 = edgeList.InsertVertex({3, 6});
        const auto v4 = edgeList.InsertVertex({3, 8});
        const auto v5 = edgeList.InsertVertex({4, 7});
        const auto v6 = edgeList.InsertVertex({5, 1});
        const auto v7 = edgeList.InsertVertex({5, 4});
        const auto v8 = edgeList.InsertVertex({5, 8});
        const auto v9 = edgeList.InsertVertex({6, 6});

        vertices.at(0) = v2;
        vertices.at(1) = v4;
        vertices.at(2) = v7;

        const auto e1 = edgeList.InsertEdge(v0, v1);
        edges.at(0) = edgeList.InsertEdge(v2, v1);
        edgeList.InsertEdge(v2, v0);

        const auto e0 = edgeList.InsertEdge(v0, v4);
        edgeList.InsertEdge(v4, v5);
        edgeList.InsertEdge(v3, v5);
        edges.at(1) = edgeList.InsertEdge(v7, v3);
        const auto e2 = edgeList.InsertEdge(v2, v7);

        const auto e3 = edgeList.InsertEdge(v2, v6);
        edgeList.InsertEdge(v6, v7);

        const auto e4 = edgeList.InsertEdge(v7, v9);
        edgeList.InsertEdge(v9, v8);
        edgeList.InsertEdge(v8, v4);
        UpdateFaces(edgeList);

        faces.at(0) = edgeList.GetFaceIndex(e0);
        faces.at(1) = edgeList.GetFaceIndex(e1);
        faces.at(2) = edgeList.GetFaceIndex(e2);
        faces.at(3) = edgeList.GetFaceIndex(e3);
        faces.at(4) = edgeList.GetFaceIndex(e4);
    }

    PointLocator locator{edgeList};

    SECTION("Vertices inside the bounding box but outside the polygons are on the unbounded face") {
        const auto location0 = locator.LocatePoint({0.5, 1.5});
        REQUIRE(location0.State.index() == 0);
        REQUIRE(std::get<0>(location0.State) == faces.at(0));

        const auto location1 = locator.LocatePoint({1.5, 8.5});
        REQUIRE(location1.State.index() == 0);
        REQUIRE(std::get<0>(location1.State) == faces.at(0));

        const auto location2 = locator.LocatePoint({2.5, 2});
        REQUIRE(location2.State.index() == 0);
        REQUIRE(std::get<0>(location2.State) == faces.at(0));

        const auto location3 = locator.LocatePoint({5.5, 3.5});
        REQUIRE(location3.State.index() == 0);
        REQUIRE(std::get<0>(location3.State) == faces.at(0));
    }

    SECTION("Vertices outside the bounding box are on the unbounded face") {
        const auto location0 = locator.LocatePoint({20, 4});
        REQUIRE(location0.State.index() == 0);
        REQUIRE(std::get<0>(location0.State) == faces.at(0));

        const auto location1 = locator.LocatePoint({2, -2});
        REQUIRE(location1.State.index() == 0);
        REQUIRE(std::get<0>(location1.State) == faces.at(0));
    }

    SECTION("Vertices on the sides and corners of the bounding box are on the unbounded face") {
        const auto location0 = locator.LocatePoint({7, 9});
        REQUIRE(location0.State.index() == 0);
        REQUIRE(std::get<0>(location0.State) == faces.at(0));

        const auto location1 = locator.LocatePoint({6, 9});
        REQUIRE(location1.State.index() == 0);
        REQUIRE(std::get<0>(location1.State) == faces.at(0));
    }

    SECTION("Vertices on bounded faces are on located on the correct face") {
        const auto location0 = locator.LocatePoint({2, 3});
        REQUIRE(location0.State.index() == 0);
        REQUIRE(std::get<0>(location0.State) == faces.at(1));

        const auto location1 = locator.LocatePoint({3, 4.5});
        REQUIRE(location1.State.index() == 0);
        REQUIRE(std::get<0>(location1.State) == faces.at(2));

        const auto location2 = locator.LocatePoint({3, 7});
        REQUIRE(location2.State.index() == 0);
        REQUIRE(std::get<0>(location2.State) == faces.at(2));

        const auto location3 = locator.LocatePoint({4.5, 6.5});
        REQUIRE(location3.State.index() == 0);
        REQUIRE(std::get<0>(location3.State) == faces.at(4));

        const auto location4 = locator.LocatePoint({3.5, 6});
        REQUIRE(location4.State.index() == 0);
        REQUIRE(std::get<0>(location4.State) == faces.at(4));
    }

    SECTION("Vertices on edges are located on the correct edge") {
        const auto location0 = locator.LocatePoint({2.5, 2.5});
        REQUIRE(location0.State.index() == 1);
        REQUIRE(std::get<1>(location0.State) == edges.at(0));

        const auto location1 = locator.LocatePoint({4, 5});
        REQUIRE(location1.State.index() == 1);
        REQUIRE(std::get<1>(location1.State) == edges.at(1));
    }

    SECTION("Vertices that coincide with vertices of the edge list are located correctly") {
        const auto location0 = locator.LocatePoint({3, 3});
        REQUIRE(location0.State.index() == 2);
        REQUIRE(std::get<2>(location0.State) == vertices.at(0));

        const auto location1 = locator.LocatePoint({3, 8});
        REQUIRE(location1.State.index() == 2);
        REQUIRE(std::get<2>(location1.State) == vertices.at(1));

        const auto location2 = locator.LocatePoint({5, 4});
        REQUIRE(location2.State.index() == 2);
        REQUIRE(std::get<2>(location2.State) == vertices.at(2));
    }
}
