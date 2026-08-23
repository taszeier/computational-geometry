#include <catch2/catch_test_macros.hpp>

#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include "voronoi/VoronoiCalculator.hpp"

using namespace compg;

TEST_CASE("Circle event coincides with a site", "[VoronoiCalculator]") {
    const std::vector<Vertex2D> vertices{{3, 8}, {5, 8}, {8, 5}, {5, 2}, {9, 6}};
    const Box2D box{{0, 0}, {10, 13}};

    VoronoiCalculator voronoiCalculator;
    const auto voronoi = voronoiCalculator.FindVoronoiDiagram(vertices, box);
    const auto voronoiOptimized = Optimize(voronoi);

    DoublyConnectedEdgeList expected;
    {
        const auto v0 = expected.InsertVertex({0, 13});
        const auto v1 = expected.InsertVertex({0, 11.0 / 3});
        const auto v2 = expected.InsertVertex({0, 0});
        const auto v3 = expected.InsertVertex({4, 13});
        const auto v4 = expected.InsertVertex({4, 5});
        const auto v5 = expected.InsertVertex({5, 5});
        const auto v6 = expected.InsertVertex({7, 7});
        const auto v7 = expected.InsertVertex({10, 13});
        const auto v8 = expected.InsertVertex({10, 4});
        const auto v9 = expected.InsertVertex({10, 0});

        expected.InsertEdge(v0, v1);
        expected.InsertEdge(v1, v2);
        expected.InsertEdge(v2, v9);
        expected.InsertEdge(v9, v8);
        expected.InsertEdge(v8, v7);
        expected.InsertEdge(v7, v3);
        expected.InsertEdge(v3, v0);
        expected.InsertEdge(v1, v4);
        expected.InsertEdge(v3, v4);
        expected.InsertEdge(v5, v4);
        expected.InsertEdge(v5, v9);
        expected.InsertEdge(v5, v6);
        expected.InsertEdge(v6, v8);
        expected.InsertEdge(v6, v7);
    }

    REQUIRE(AreIsomorphic(voronoiOptimized, expected));
}

TEST_CASE("Two Circle events coincide", "[VoronoiCalculator]") {
    const std::vector<Vertex2D> vertices{{2, 2}, {3, 1}, {4, 2}, {3, 3}};
    const Box2D box{{1, 0}, {5, 4}};

    VoronoiCalculator voronoiCalculator;
    const auto voronoi = voronoiCalculator.FindVoronoiDiagram(vertices, box);
    const auto voronoiOptimized = Optimize(voronoi);

    DoublyConnectedEdgeList expected;
    {
        const auto v0 = expected.InsertVertex({1, 0});
        const auto v1 = expected.InsertVertex({5, 0});
        const auto v2 = expected.InsertVertex({5, 4});
        const auto v3 = expected.InsertVertex({1, 4});
        const auto v4 = expected.InsertVertex({3, 2});

        expected.InsertEdge(v0, v1);
        expected.InsertEdge(v1, v2);
        expected.InsertEdge(v2, v3);
        expected.InsertEdge(v3, v0);
        expected.InsertEdge(v0, v4);
        expected.InsertEdge(v1, v4);
        expected.InsertEdge(v2, v4);
        expected.InsertEdge(v3, v4);
    }

    REQUIRE(AreIsomorphic(voronoiOptimized, expected));
}

TEST_CASE("A voronoi cell is bounded", "[VoronoiCalculator]") {
    const std::vector<Vertex2D> vertices{{1, 8}, {1, 5}, {3, 2}, {5, 6}, {9, 10}, {9, 2}};
    const Box2D box{{0, 0}, {10, 13}};

    VoronoiCalculator voronoiCalculator;
    const auto voronoi = voronoiCalculator.FindVoronoiDiagram(vertices, box);
    const auto voronoiOptimized = Optimize(voronoi);

    DoublyConnectedEdgeList expected;
    {
        const auto v0 = expected.InsertVertex({0, 13});
        const auto v1 = expected.InsertVertex({0, 6.5});
        const auto v2 = expected.InsertVertex({0, 0});
        const auto v3 = expected.InsertVertex({2.75, 6.5});
        const auto v4 = expected.InsertVertex({23.0 / 7, 6 - 23.0 / 14});
        const auto v5 = expected.InsertVertex({4, 13});
        const auto v6 = expected.InsertVertex({14.0 / 3, 31.0 / 3});

        const auto v7 = expected.InsertVertex({6, 3});
        const auto v8 = expected.InsertVertex({6, 0});
        const auto v9 = expected.InsertVertex({9, 6});
        const auto v10 = expected.InsertVertex({10, 13});
        const auto v11 = expected.InsertVertex({10, 6});
        const auto v12 = expected.InsertVertex({10, 0});
        const auto v13 = expected.InsertVertex({0, 13.0 / 6});

        expected.InsertEdge(v0, v1);
        expected.InsertEdge(v1, v13);
        expected.InsertEdge(v13, v2);
        expected.InsertEdge(v2, v8);
        expected.InsertEdge(v8, v12);
        expected.InsertEdge(v12, v11);
        expected.InsertEdge(v11, v10);
        expected.InsertEdge(v10, v5);
        expected.InsertEdge(v5, v0);

        expected.InsertEdge(v1, v3);
        expected.InsertEdge(v13, v4);
        expected.InsertEdge(v5, v6);
        expected.InsertEdge(v3, v4);
        expected.InsertEdge(v3, v6);
        expected.InsertEdge(v7, v4);
        expected.InsertEdge(v7, v8);
        expected.InsertEdge(v7, v9);
        expected.InsertEdge(v11, v9);
        expected.InsertEdge(v6, v9);
    }

    REQUIRE(AreIsomorphic(voronoiOptimized, expected));
}