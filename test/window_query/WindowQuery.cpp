#include <catch2/catch_test_macros.hpp>

#include "window_query/WindowQuery.hpp"

using namespace compg;

TEST_CASE("WindowQuery", "[WindowQuery]") {
    std::vector<LineSegment2D> segments{{{5, 8}, {2, 2}}, {{2, 1}, {4, 3}},  {{5, 6}, {5, 1}}, {{5.5, 4}, {7, 4}},
                                        {{3, 1}, {4, 1}}, {{3, 10}, {5, 8}}, {{7, 1}, {8, 3}}, {{2, 3}, {1, 5}}};

    const WindowQuery query{segments};
    auto result = query.Query({{3, 2}, {6, 5}});
    std::ranges::sort(result);

    REQUIRE(result.size() == 4);
    REQUIRE(result.at(0) == 0);
    REQUIRE(result.at(1) == 1);
    REQUIRE(result.at(2) == 2);
    REQUIRE(result.at(3) == 3);
}