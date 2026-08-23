#include <catch2/catch_test_macros.hpp>

#include "linear_programming/MinimumDiscCalculator.hpp"

using namespace compg;

TEST_CASE("The disc contains two vertices", "[MinimumDiscCalculator]") {
    const std::vector<Vertex2D> vertices{{0, 0}, {3, 4}};

    MinimumDiscCalculator calculator;
    const auto minimumDisc = calculator.FindMinimumDisc(vertices);

    REQUIRE(minimumDisc == Ball<2UZ>{{1.5, 2}, 2.5});
}

TEST_CASE("The disc contains three non-collinear vertices", "[MinimumDiscCalculator]") {
    const std::vector<Vertex2D> vertices{{0, 0}, {3, 4}, {1, 3}};

    MinimumDiscCalculator calculator;
    const auto minimumDisc = calculator.FindMinimumDisc(vertices);

    REQUIRE(minimumDisc == Ball<2UZ>{{1.5, 2}, 2.5});
}

TEST_CASE("The disc contains three collinear vertices", "[MinimumDiscCalculator]") {
    const std::vector<Vertex2D> vertices{{0, 0}, {4, 0}, {1, 0}};

    MinimumDiscCalculator calculator;
    const auto minimumDisc = calculator.FindMinimumDisc(vertices);

    REQUIRE(minimumDisc == Ball<2UZ>{{2, 0}, 2});
}

TEST_CASE("The disc contains more than three vertices", "[MinimumDiscCalculator]") {
    const std::vector<Vertex2D> vertices{{-2 * std::sqrt(2), 0},
                                         {-2, -1},
                                         {-2, 0},
                                         {-1, 1},
                                         {0, -2},
                                         {0, 2},
                                         {1, -1},
                                         {1, 0},
                                         {1, 1},
                                         {2, -2},
                                         {2, 0},
                                         {2, 2}};

    MinimumDiscCalculator calculator;
    const auto minimumDisc = calculator.FindMinimumDisc(vertices);

    const Ball<2UZ> expected{{0, 0}, 2 * std::sqrt(2)};
    REQUIRE(AreEqual(minimumDisc, expected));
}