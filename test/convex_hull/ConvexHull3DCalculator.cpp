#include <catch2/catch_test_macros.hpp>

#include "convex_hull/ConvexHull3DCalculator.hpp"

using namespace compg;

TEST_CASE("ConvexHull3DCalculator", "[ConvexHull3DCalculator]") {

    SECTION("Adjacent horizon edges bound the same face") {
        const std::vector<Vertex3D> vertices{{0, 0, 0}, {0, 0, 2}, {0, 2, 0}, {3, 0, 0}, {2, 0, 3}, {3, 0, 4}};
        const ConvexHull3DCalculator calculator{};
        const auto hull = calculator.FindConvexHull(vertices);

        const auto expected = CreateEdgeList(
            {{0, 0, 0}, {0, 0, 2}, {0, 2, 0}, {3, 0, 0}, {3, 0, 4}},
            {{0, 2, 3}, {1, 0, 3, 4}, {0, 1, 2}, {1, 4, 2}, {3, 2, 4}}
        );

        REQUIRE(AreIsomorphic(hull.EdgeList, expected));
    }

    SECTION("The initial tetrahedron is inside the convex hull") {
        const std::vector<Vertex3D> vertices{{0, 0, 0},     {0, 0, 2},     {0, 2, 0},       {1, 1, 1},
                                             {3, 0, 0},     {2, 0, 3},     {3, 0, 4},       {5, 2, 3},
                                             {-3, -2, 0},   {-5, -5, -5},  {-10, -10, 10},  {10, -10, 10},
                                             {10, 10, 10},  {-10, 10, 10}, {-10, -10, -10}, {10, -10, -10},
                                             {10, 10, -10}, {-10, 10, -10}};
        const ConvexHull3DCalculator calculator{};
        const auto hull = calculator.FindConvexHull(vertices);

        const auto expected = CreateEdgeList(
            {{-10, -10, 10},
             {10, -10, 10},
             {10, 10, 10},
             {-10, 10, 10},
             {-10, -10, -10},
             {10, -10, -10},
             {10, 10, -10},
             {-10, 10, -10}},
            {{0, 1, 2, 3}, {4, 7, 6, 5}, {0, 4, 5, 1}, {1, 5, 6, 2}, {2, 6, 7, 3}, {3, 7, 4, 0}}
        );
        REQUIRE(AreIsomorphic(hull.EdgeList, expected));
    }
}