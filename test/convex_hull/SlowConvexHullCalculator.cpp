#include <catch2/catch_test_macros.hpp>

#include "convex_hull/SlowConvexHullCalculator.hpp"
#include "math/Math.hpp"

using namespace compg;

TEST_CASE("Slow convex hull calculator", "[SlowConvexHullCalculator]") {
    const std::vector<Vertex2D> vertices{{0, 0}, {0.5, 0.5}, {0.1, 0.9}, {1, 1}, {0, 1}, {1, 0}, {0.5, 5}};

    const SlowConvexHullCalculator calculator;
    const auto hullVertices = calculator.FindConvexHull(vertices).Vertices;

    REQUIRE(hullVertices.size() == 5);
    REQUIRE(AreEqual(hullVertices.at(0), {0, 0}));
    REQUIRE(AreEqual(hullVertices.at(1), {0, 1}));
    REQUIRE(AreEqual(hullVertices.at(2), {0.5, 5}));
    REQUIRE(AreEqual(hullVertices.at(3), {1, 1}));
    REQUIRE(AreEqual(hullVertices.at(4), {1, 0}));
}