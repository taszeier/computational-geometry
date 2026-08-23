#include <catch2/catch_test_macros.hpp>

#include "math/Math.hpp"
#include "window_query/SegmentTree.hpp"

using namespace compg;

TEST_CASE("Horizontal Segment Tree", "[SegmentTree]") {
    std::vector<LineSegment2D> segments{{{1, 3}, {4, 4}}, {{2, 5}, {5, 5}}, {{2, 10}, {9, 8}},
                                        {{4, 2}, {7, 1}}, {{5, 5}, {5, 3}}, {{6, 4}, {6, 8}}};
    const HorizontalSegmentTree tree{segments};

    SECTION("The query region does not intersect any segments") {
        const auto result = tree.Query({Interval<Float>{4, 10}, 0});
        REQUIRE(result.empty());
    }

    SECTION("The query region intersects an endpoint of a segment") {
        const auto result = tree.Query({Interval<Float>{4, 10}, 1});
        REQUIRE(result.size() == 1);
        REQUIRE(result.at(0).Index == 0);
        REQUIRE(AreEqual(result.at(0).Segment, segments.at(0)));
    }

    SECTION("The query region intersects multiple segments") {
        auto result = tree.Query({Interval<Float>{2, 10}, 3});
        std::ranges::sort(result, Less{}, IndexProjector{});

        REQUIRE(result.size() == 3);
        REQUIRE(result.at(0).Index == 0);
        REQUIRE(AreEqual(result.at(0).Segment, segments.at(0)));
        REQUIRE(result.at(1).Index == 1);
        REQUIRE(AreEqual(result.at(1).Segment, segments.at(1)));
        REQUIRE(result.at(2).Index == 2);
        REQUIRE(AreEqual(result.at(2).Segment, segments.at(2)));
    }

    SECTION("The query region is above and below two segments") {
        auto result = tree.Query({Interval<Float>{4.5, 6}, 3});

        REQUIRE(result.size() == 1);
        REQUIRE(result.at(0).Index == 1);
        REQUIRE(AreEqual(result.at(0).Segment, segments.at(1)));
    }

    SECTION("The query region covers a vertical segment") {
        auto result = tree.Query({Interval<Float>{2, 7}, 5});
        std::ranges::sort(result, Less{}, IndexProjector{});

        REQUIRE(result.size() == 2);
        REQUIRE(result.at(0).Index == 1);
        REQUIRE(AreEqual(result.at(0).Segment, segments.at(1)));
        REQUIRE(result.at(1).Index == 4);
        REQUIRE(AreEqual(result.at(1).Segment, segments.at(4)));
    }

    SECTION("The query region intersects a vertical segment") {
        auto result = tree.Query({Interval<Float>{1, 5}, 6});
        std::ranges::sort(result, Less{}, IndexProjector{});

        REQUIRE(result.size() == 2);
        REQUIRE(result.at(0).Index == 3);
        REQUIRE(AreEqual(result.at(0).Segment, segments.at(3)));
        REQUIRE(result.at(1).Index == 5);
        REQUIRE(AreEqual(result.at(1).Segment, segments.at(5)));
    }
}