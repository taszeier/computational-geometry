#include <catch2/catch_test_macros.hpp>

#include "convex_hull/AndrewsMonotoneChain.hpp"
#include "math/Math.hpp"

using namespace compg;

TEST_CASE("The lower and upper convex hulls are different", "[AndrewsMonotoneChain]") {
    const std::vector<Vertex2D> vertices{{0, 0}, {0.5, 0.5}, {0.1, 0.9}, {1, 1}, {0, 1}, {1, 0}, {0.5, 5}};

    const AndrewsMonotoneChain calculator;
    const auto hullVertices = calculator.FindConvexHull(vertices).Vertices;

    REQUIRE(hullVertices.size() == 5);
    REQUIRE(AreEqual(hullVertices.at(0), {0, 0}));
    REQUIRE(AreEqual(hullVertices.at(1), {0, 1}));
    REQUIRE(AreEqual(hullVertices.at(2), {0.5, 5}));
    REQUIRE(AreEqual(hullVertices.at(3), {1, 1}));
    REQUIRE(AreEqual(hullVertices.at(4), {1, 0}));
}

TEST_CASE("The lower and upper convex hulls are the same", "[AndrewsMonotoneChain]") {
    const std::vector<Vertex2D> vertices{{2, 0}, {1, 0}, {0, 0}, {5, 0}, {4, 0}};

    const AndrewsMonotoneChain calculator;
    const auto hullVertices = calculator.FindConvexHull(vertices).Vertices;

    const std::vector<Vertex2D> expected{{0, 0}, {5, 0}};
    REQUIRE(AreEqual(hullVertices, expected));
}