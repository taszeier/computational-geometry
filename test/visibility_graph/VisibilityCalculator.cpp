#include <catch2/catch_test_macros.hpp>

#include "visibility_graph/VisibilityCalculator.hpp"

using namespace compg;

TEST_CASE("VisibilityCalculator", "[VisibilityCalculator]") {
    const Polygon p1{{{5, 2}, {7, 1}, {9, 2}, {8, 3}, {7, 2}, {6, 3}}};
    const Polygon p2{
        {{9, 7}, {8, 6}, {8, 4}, {9, 4}, {9, 6}, {10, 4}, {10, 3}, {12, 3}, {12, 4}, {12, 5}, {13, 6}, {14, 3}, {14, 8}}
    };
    const std::vector obstacles{p1, p2};
    const VisibilityCalculator calculator;
    const LexicographicalLess<Vertex2D> less;

    SECTION("The origin is outside the obstacles") {
        auto result = calculator.FindVisibleVertices({7, 4}, obstacles);
        std::ranges::sort(result, less);

        const std::vector<Vertex2D> expected{{5, 2}, {6, 3}, {7, 2}, {8, 3},  {8, 4},
                                             {8, 6}, {9, 2}, {9, 4}, {10, 3}, {10, 4}};

        CHECK(AreEqual(result, expected));
    }

    SECTION("The origin is a vertex of an obstacle") {
        auto result = calculator.FindVisibleVertices({12, 5}, obstacles);
        std::ranges::sort(result, less);

        const std::vector<Vertex2D> expected{{12, 3}, {12, 4}, {13, 6}, {14, 3}};

        CHECK(AreEqual(result, expected));
    }

    SECTION("The origin is on the edge of an obstacle") {
        auto result = calculator.FindVisibleVertices({7.5, 2.5}, obstacles);
        std::ranges::sort(result, less);

        const std::vector<Vertex2D> expected{{6, 3}, {7, 2}, {8, 3}, {8, 4}, {8, 6}, {9, 4}};

        CHECK(AreEqual(result, expected));
    }
}