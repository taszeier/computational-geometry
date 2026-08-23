#include <catch2/catch_test_macros.hpp>

#include "range_query/kd_tree/BallQueryRegion.hpp"

using namespace compg;

TEST_CASE("BallQueryRegion - query region contains vertex", "[BallQueryRegion]") {
    const BallQueryRegion<2> queryRegion{{{0, 0}, 1}};

    SECTION("Query region contains vertex in the interior of the region") {
        REQUIRE(queryRegion.Contains({0.5, 0.5}));
        REQUIRE(queryRegion.Contains({-0.75, 0.6}));
    }

    SECTION("Query region contains vertex on the boundary of the region") {
        REQUIRE(queryRegion.Contains({0, 1}));
        REQUIRE(queryRegion.Contains({1, 0}));
        REQUIRE(queryRegion.Contains({-0.5, std::sqrt(3) * 0.5}));
    }

    SECTION("Query region does not contain vertex outside of the region") {
        REQUIRE_FALSE(queryRegion.Contains({0, 2}));
        REQUIRE_FALSE(queryRegion.Contains({-3, -4}));
    }
}

TEST_CASE("BallQueryRegion - query region covers KdTree region", "[BallQueryRegion]") {
    const BallQueryRegion<2> queryRegion{{{0, 0}, 1}};

    SECTION("Query region covers KdTree region inside its interior") {
        KdTreeRegion<2> treeRegion;
        treeRegion.Intersect(AxesAlignedHyperplane<2>{0, -0.5}, Side::Positive);
        treeRegion.Intersect(AxesAlignedHyperplane<2>{0, 0.5}, Side::Negative);
        treeRegion.Intersect(AxesAlignedHyperplane<2>{1, -0.5}, Side::Positive);
        treeRegion.Intersect(AxesAlignedHyperplane<2>{1, 0.5}, Side::Negative);

        REQUIRE(queryRegion.Covers(treeRegion));
    }

    SECTION("Query region covers KdTree region intersecting its boundary") {
        KdTreeRegion<2> treeRegion;
        treeRegion.Intersect(AxesAlignedHyperplane<2>{0, -0.5}, Side::Positive);
        treeRegion.Intersect(AxesAlignedHyperplane<2>{0, 0.5}, Side::Negative);
        treeRegion.Intersect(AxesAlignedHyperplane<2>{1, -std::sqrt(3) * 0.5}, Side::Positive);
        treeRegion.Intersect(AxesAlignedHyperplane<2>{1, std::sqrt(3) * 0.5}, Side::Negative);

        REQUIRE(queryRegion.Covers(treeRegion));
    }

    SECTION("Query region does not cover unbounded KdTree region") {
        KdTreeRegion<2> treeRegion;
        treeRegion.Intersect(AxesAlignedHyperplane<2>{0, -0.5}, Side::Positive);
        treeRegion.Intersect(AxesAlignedHyperplane<2>{1, -0.5}, Side::Positive);

        REQUIRE_FALSE(queryRegion.Covers(treeRegion));
    }

    SECTION("Query region does not cover disjoint KdTree region") {
        KdTreeRegion<2> treeRegion;
        treeRegion.Intersect(AxesAlignedHyperplane<2>{0, 2}, Side::Positive);
        treeRegion.Intersect(AxesAlignedHyperplane<2>{0, 3}, Side::Negative);
        treeRegion.Intersect(AxesAlignedHyperplane<2>{1, 0}, Side::Positive);
        treeRegion.Intersect(AxesAlignedHyperplane<2>{1, 1}, Side::Negative);

        REQUIRE_FALSE(queryRegion.Covers(treeRegion));
    }

    SECTION("Query region covers empty KdTree region") {
        KdTreeRegion<2> treeRegion;
        treeRegion.Intersect(AxesAlignedHyperplane<2>{0, 2}, Side::Positive);
        treeRegion.Intersect(AxesAlignedHyperplane<2>{0, 1}, Side::Negative);

        REQUIRE(queryRegion.Covers(treeRegion));
    }
}

TEST_CASE("BallQueryRegion - query intersects KdTree region", "[BallQueryRegion]") {
    const BallQueryRegion<2> queryRegion{{{0, 0}, 1}};

    SECTION("Query region intersects KdTree region that it covers") {
        KdTreeRegion<2> treeRegion;
        treeRegion.Intersect(AxesAlignedHyperplane<2>{0, -0.5}, Side::Positive);
        treeRegion.Intersect(AxesAlignedHyperplane<2>{0, 0.5}, Side::Negative);
        treeRegion.Intersect(AxesAlignedHyperplane<2>{1, -0.5}, Side::Positive);
        treeRegion.Intersect(AxesAlignedHyperplane<2>{1, 0.5}, Side::Negative);

        REQUIRE(queryRegion.Intersects(treeRegion));
    }

    SECTION("Query region intersects KdTree region that covers it") {
        KdTreeRegion<2> treeRegion;
        treeRegion.Intersect(AxesAlignedHyperplane<2>{0, -5}, Side::Positive);
        treeRegion.Intersect(AxesAlignedHyperplane<2>{0, 5}, Side::Negative);
        treeRegion.Intersect(AxesAlignedHyperplane<2>{1, -5}, Side::Positive);
        treeRegion.Intersect(AxesAlignedHyperplane<2>{1, 5}, Side::Negative);

        REQUIRE(queryRegion.Intersects(treeRegion));
    }

    SECTION("Query region intersects KdTree region only on its boundary") {
        KdTreeRegion<2> treeRegion;
        treeRegion.Intersect(AxesAlignedHyperplane<2>{0, 1}, Side::Positive);

        REQUIRE(queryRegion.Intersects(treeRegion));
    }

    SECTION(
        "Query region intersects KdTree region where the differences aren't "
        "empty"
    ) {
        KdTreeRegion<2> treeRegion;
        treeRegion.Intersect(AxesAlignedHyperplane<2>{0, 0}, Side::Positive);

        REQUIRE(queryRegion.Intersects(treeRegion));
    }

    SECTION("Query region does not intersect empty KdTree region") {
        KdTreeRegion<2> treeRegion;
        treeRegion.Intersect(AxesAlignedHyperplane<2>{0, 2}, Side::Positive);
        treeRegion.Intersect(AxesAlignedHyperplane<2>{0, 1}, Side::Negative);

        REQUIRE_FALSE(queryRegion.Intersects(treeRegion));
    }

    SECTION(
        "Query region does not intersect non-empty KdTree region outside the "
        "region"
    ) {
        KdTreeRegion<2> treeRegion;
        treeRegion.Intersect(AxesAlignedHyperplane<2>{0, 2}, Side::Positive);
        treeRegion.Intersect(AxesAlignedHyperplane<2>{1, 2}, Side::Negative);

        REQUIRE_FALSE(queryRegion.Intersects(treeRegion));
    }
}
