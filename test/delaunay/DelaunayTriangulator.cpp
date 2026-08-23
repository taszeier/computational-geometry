#include <catch2/catch_test_macros.hpp>

#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include "delaunay/DelaunayTriangulator.hpp"

using namespace compg;

TEST_CASE("Triangulate zero vertices", "[DelaunayTriangulator]") {
    const std::vector<Vertex2D> vertices;

    DelaunayTriangulator triangulator;
    auto triangulation = triangulator.Triangulate(vertices);
    UpdateFaces(triangulation);

    DoublyConnectedEdgeList expected;
    UpdateFaces(expected);

    REQUIRE(AreIsomorphic(triangulation, expected));
}

TEST_CASE("Triangulate one vertex", "[DelaunayTriangulator]") {
    const std::vector<Vertex2D> vertices{{4, -2}};

    DelaunayTriangulator triangulator;
    auto triangulation = triangulator.Triangulate(vertices);
    UpdateFaces(triangulation);

    DoublyConnectedEdgeList expected;
    expected.InsertVertex({4, -2});

    UpdateFaces(expected);

    REQUIRE(AreIsomorphic(triangulation, expected));
}

TEST_CASE("Triangulate two vertices", "[DelaunayTriangulator]") {
    const std::vector<Vertex2D> vertices{{4, -2}, {10, 10}};

    DelaunayTriangulator triangulator;
    auto triangulation = triangulator.Triangulate(vertices);
    UpdateFaces(triangulation);

    DoublyConnectedEdgeList expected;
    {
        const auto v0 = expected.InsertVertex({4, -2});
        const auto v1 = expected.InsertVertex({10, 10});
        expected.InsertEdge(v0, v1);
    }

    UpdateFaces(expected);

    REQUIRE(AreIsomorphic(triangulation, expected));
}

TEST_CASE("Three vertices create a single triangle", "[DelaunayTriangulator]") {
    const std::vector<Vertex2D> vertices{{0, 0}, {1, 3}, {4, -2}};

    DelaunayTriangulator triangulator;
    auto triangulation = triangulator.Triangulate(vertices);
    UpdateFaces(triangulation);

    DoublyConnectedEdgeList expected;
    {
        const auto v0 = expected.InsertVertex({0, 0});
        const auto v1 = expected.InsertVertex({4, -2});
        const auto v2 = expected.InsertVertex({1, 3});
        expected.InsertEdge(v0, v1);
        expected.InsertEdge(v0, v2);
        expected.InsertEdge(v1, v2);
    }

    UpdateFaces(expected);

    REQUIRE(AreIsomorphic(triangulation, expected));
}

TEST_CASE("Multiple non-collinear vertices", "[DelaunayTriangulator]") {
    const std::vector<Vertex2D> vertices{{0, 0}, {2, 2}, {2, -5}, {4, 0}, {4, -4}};

    DelaunayTriangulator triangulator;
    auto triangulation = triangulator.Triangulate(vertices);
    UpdateFaces(triangulation);

    DoublyConnectedEdgeList expected;
    {
        const auto v0 = expected.InsertVertex({0, 0});
        const auto v1 = expected.InsertVertex({2, 2});
        const auto v2 = expected.InsertVertex({2, -5});
        const auto v3 = expected.InsertVertex({4, 0});
        const auto v4 = expected.InsertVertex({4, -4});
        expected.InsertEdge(v0, v1);
        expected.InsertEdge(v0, v2);
        expected.InsertEdge(v0, v3);
        expected.InsertEdge(v3, v1);
        expected.InsertEdge(v3, v4);
        expected.InsertEdge(v2, v4);
        expected.InsertEdge(v0, v4);
    }

    UpdateFaces(expected);

    REQUIRE(AreIsomorphic(triangulation, expected));
}

TEST_CASE("Collinear vertices", "[DelaunayTriangulator]") {
    const std::vector<Vertex2D> vertices{{1, 2}, {1, 4}, {3, 1}, {3, 2}, {3, 3}, {3, 4}, {5, 2}, {5, 4}};

    DelaunayTriangulator triangulator;
    auto triangulation = triangulator.Triangulate(vertices);
    UpdateFaces(triangulation);

    DoublyConnectedEdgeList expected;
    {
        const auto vi = vertices
                        | std::views::transform([&expected](const auto& v) { return expected.InsertVertex(v); })
                        | std::ranges::to<std::vector>();
        expected.InsertEdge(vi[0], vi[1]);
        expected.InsertEdge(vi[0], vi[2]);
        expected.InsertEdge(vi[0], vi[3]);
        expected.InsertEdge(vi[0], vi[4]);
        expected.InsertEdge(vi[1], vi[4]);
        expected.InsertEdge(vi[1], vi[5]);
        expected.InsertEdge(vi[2], vi[3]);
        expected.InsertEdge(vi[2], vi[6]);
        expected.InsertEdge(vi[3], vi[4]);
        expected.InsertEdge(vi[3], vi[6]);

        expected.InsertEdge(vi[4], vi[5]);
        expected.InsertEdge(vi[4], vi[6]);
        expected.InsertEdge(vi[4], vi[7]);
        expected.InsertEdge(vi[5], vi[7]);
        expected.InsertEdge(vi[6], vi[7]);
    }

    UpdateFaces(expected);

    REQUIRE(AreIsomorphic(triangulation, expected));
}