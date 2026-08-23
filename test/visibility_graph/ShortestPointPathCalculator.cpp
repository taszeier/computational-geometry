#include <catch2/catch_test_macros.hpp>

#include "visibility_graph/Common.hpp"
#include "visibility_graph/ShortestPointPathCalculator.hpp"

using namespace compg;

TEST_CASE("ShortestPointPathCalculator", "[ShortestPointPathCalculator]") {
    const Polygon p1{{{5, 2}, {7, 1}, {9, 2}, {8, 3}, {7, 2}, {6, 3}}};
    const Polygon p2{
        {{9, 7}, {8, 6}, {8, 4}, {9, 4}, {9, 6}, {10, 4}, {10, 3}, {12, 3}, {12, 4}, {12, 5}, {13, 6}, {14, 3}, {14, 8}}
    };
    const Polygon p3{{{1, 0}, {2, 0}, {2, 20}, {1, 20}}};
    const Box2D box{{0, 0}, {20, 20}};
    const std::vector obstacles{p1, p2, p3};
    const ShortestPointPathCalculator calculator{box, obstacles};

    SECTION("The shortest path is a straight line") {
        const auto path = calculator.FindPath({4, 1}, {7, 4});
        const std::vector<Vertex2D> expected{{4, 1}, {7, 4}};

        REQUIRE(path.has_value());
        CHECK(AreEqual(path.value(), expected));
    }

    SECTION("The path goes around an obstacle") {
        const auto path = calculator.FindPath({4, 1}, {13, 4});
        const std::vector<Vertex2D> expected{{4, 1}, {7, 1}, {12, 3}, {13, 4}};

        REQUIRE(path.has_value());
        CHECK(AreEqual(path.value(), expected));
    }

    SECTION("The path goes along an obstacle edge") {
        const auto path = calculator.FindPath({6, 1}, {9, 3});
        const std::vector<Vertex2D> expected{{6, 1}, {7, 1}, {9, 2}, {9, 3}};

        REQUIRE(path.has_value());
        CHECK(AreEqual(path.value(), expected));
    }

    SECTION("The start point is in the interior of an obstacle") {
        const auto path = calculator.FindPath({11, 5}, {9, 3});
        CHECK_FALSE(path.has_value());
    }

    SECTION("The goal point is in the interior of an obstacle") {
        const auto path = calculator.FindPath({4, 1}, {6, 2});
        CHECK_FALSE(path.has_value());
    }

    SECTION("There is no path from start to goal") {
        const auto path = calculator.FindPath({6, 1}, {0.5, 2});
        CHECK_FALSE(path.has_value());
    }
}

TEST_CASE("Shortest path does not go through obstacles", "[ShortestPointPathCalculator]") {
    const Polygon p1{{{2, 4}, {4, 2}, {6, 2}, {6, 4}, {4, 6}}};
    const Polygon p2{{{2, 1}, {2, 0}, {6, 0}, {6, 1}}};
    const Box2D box{{0, 0}, {10, 10}};
    const ShortestPointPathCalculator calculator{box, {p1, p2}};

    SECTION("The path does not go through the obstacle when the start and goal points are on the edge") {
        const auto path = calculator.FindPath({3, 5}, {5, 5});
        const std::vector<Vertex2D> expected{{3, 5}, {4, 6}, {5, 5}};

        REQUIRE(path.has_value());
        CHECK(AreEqual(path.value(), expected));
    }

    SECTION("The path does not go through the obstacles at its vertices") {
        const auto path = calculator.FindPath({1, 4}, {7, 4});
        const std::vector<Vertex2D> expected{{1, 4}, {4, 6}, {7, 4}};

        REQUIRE(path.has_value());
        CHECK(AreEqual(path.value(), expected));
    }

    SECTION("The path does not follow an obstacle edge on the boundary of the box") {
        const auto path = calculator.FindPath({1, 0}, {7, 0});
        const std::vector<Vertex2D> expected{{1, 0}, {2, 1}, {6, 1}, {7, 0}};

        REQUIRE(path.has_value());
        CHECK(AreEqual(path.value(), expected));
    }
}