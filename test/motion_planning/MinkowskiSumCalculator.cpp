#include <catch2/catch_test_macros.hpp>

#include "motion_planning/MinkowskiSumCalculator.hpp"

using namespace compg;

TEST_CASE("FindConvexSum", "[MinkowskiSumCalculator]") {
    const Polygon p1{{{0, 0}, {2, 0}, {1, 2}}};

    const Polygon p2{{{0, 1}, {1, 0}, {2, 0}, {3, 1}, {2, 3}, {1, 3}}};

    const MinkowskiSumCalculator calculator;
    const auto result = calculator.FindConvexSum(p1, p2);

    const Polygon expected{{{3, 5}, {2, 5}, {0, 1}, {1, 0}, {4, 0}, {5, 1}}};

    REQUIRE(AreEqual(result, expected));
}

TEST_CASE("FindHalfConvexSum", "[MinkowskiSumCalculator]") {
    const Polygon p1{{{2, 6}, {3, 4}, {2, 2}, {4, 2}, {4, 6}}};

    const Polygon p2{{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}};

    const MinkowskiSumCalculator calculator;
    const auto result = calculator.FindHalfConvexSum(p1, p2);
    const auto optimized = Optimize(result);

    const Polygon expected{{{1, 7}, {1, 5}, {1.5, 4}, {1, 3}, {1, 1}, {5, 1}, {5, 7}}};

    REQUIRE(AreEqual(optimized, expected));
}

TEST_CASE("FindMinkowskiSum", "[MinkowskiSumCalculator]") {
    SECTION("Two non-convex polygons") {
        const Polygon p1{{{0, 0}, {5, 0}, {3, 2}, {5, 4}, {0, 4}, {2, 2}}};
        const Polygon p2{{{0, 0}, {4, 0}, {3, 2}, {2, 1}, {1, 2}}};

        const MinkowskiSumCalculator calculator;
        const auto result = calculator.FindMinkowskiSum(p1, p2);
        const auto optimized = Optimize(result);

        const Polygon expected{
            {{0, 0}, {9, 0}, {8, 2}, {7.5, 2.5}, {9, 4}, {8, 6}, {1, 6}, {0, 4}, {1.5, 2.5}, {1, 2}}
        };

        REQUIRE(AreEqual(optimized, expected));
    }

    SECTION("One non-convex polygon") {
        const Polygon p1{{{2, 0}, {4, 2}, {3, 3}, {4, 4}, {2, 6}, {0, 4}, {1, 3}, {0, 2}}};

        const Polygon p2{{{0, 0}, {2, 0}, {2, 2}, {1, 1}, {0, 2}}};

        const MinkowskiSumCalculator calculator;
        const auto result = calculator.FindMinkowskiSum(p1, p2);
        const auto optimized = Optimize(result);

        const Polygon expected{{{2, 0}, {4, 0}, {6, 2}, {6, 6}, {4, 8}, {3, 7}, {2, 8}, {0, 6}, {0, 2}}};

        REQUIRE(AreEqual(optimized, expected));
    }
}