#include <catch2/catch_test_macros.hpp>

#include "window_query/AxesAlignedWindowQuery.hpp"

using namespace compg;

TEST_CASE("AxesAlignedWindowQuery", "[AxesAlignedWindowQuery]") {
    const std::vector<LineSegment2D> segments{{{1, 5}, {3, 5}}, {{4, 8}, {4, 1}}, {{5, 6}, {5, 9}},
                                              {{6, 7}, {2, 7}}, {{2, 3}, {5, 3}}, {{100, 0}, {100, 1}}};

    AxesAlignedWindowQuery query(segments);

    SECTION("Segments inside the box are found") {
        auto indices = query.Query({{0, 0}, {10, 10}});
        std::ranges::sort(indices);

        REQUIRE(indices.size() == 5);
        REQUIRE(indices.at(0) == 0);
        REQUIRE(indices.at(1) == 1);
        REQUIRE(indices.at(2) == 2);
        REQUIRE(indices.at(3) == 3);
        REQUIRE(indices.at(4) == 4);
    }

    SECTION("Segments with only one endpoint inside the query region are found") {
        auto indices = query.Query({{1.5, 0}, {4.5, 7.5}});
        std::ranges::sort(indices);

        REQUIRE(indices.size() == 4);
        REQUIRE(indices.at(0) == 0);
        REQUIRE(indices.at(1) == 1);
        REQUIRE(indices.at(2) == 3);
        REQUIRE(indices.at(3) == 4);
    }

    SECTION("Segments on the boundary of the query region are found") {
        auto indices = query.Query({{2, 3}, {7, 7}});
        std::ranges::sort(indices);

        REQUIRE(indices.size() == 5);
        REQUIRE(indices.at(0) == 0);
        REQUIRE(indices.at(1) == 1);
        REQUIRE(indices.at(2) == 2);
        REQUIRE(indices.at(3) == 3);
        REQUIRE(indices.at(4) == 4);
    }

    SECTION("Segments with no endpoints inside the query region that intersect the interior of the sides are found") {
        auto indices = query.Query({{3.5, 2.5}, {4.5, 3.5}});
        std::ranges::sort(indices);

        REQUIRE(indices.size() == 2);
        REQUIRE(indices.at(0) == 1);
        REQUIRE(indices.at(1) == 4);
    }

    SECTION("Segments with no endpoints inside the query region that contain one of the sides are found") {
        auto indices = query.Query({{3.5, 6}, {4.5, 7}});
        std::ranges::sort(indices);

        REQUIRE(indices.size() == 2);
        REQUIRE(indices.at(0) == 1);
        REQUIRE(indices.at(1) == 3);
    }
}