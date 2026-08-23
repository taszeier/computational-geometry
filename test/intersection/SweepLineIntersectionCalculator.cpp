#include <catch2/catch_test_macros.hpp>

#include "intersection/SweepLineIntersectionCalculator.hpp"

#include "common/Algorithms.hpp"
#include "math/Math.hpp"
#include "sweep_line/SweepLine.hpp"

using namespace compg;

TEST_CASE("Two lines intersect in the middle", "[SweepLineIntersectionCalculator]") {
    const std::vector<LineSegment2D> lines{{{0, 0}, {4, 4}}, {{2, 3}, {2, -4}}};
    SweepLineIntersectionCalculator calculator{};
    const auto result = calculator.FindIntersections(lines);
    const auto& intersections = result.Intersections;
    const auto& segments = result.IntersectingSegments;

    REQUIRE(intersections.size() == 1);
    REQUIRE(AreEqual(intersections[0], {2, 2}));

    REQUIRE(segments.size() == 1);
    REQUIRE(segments[0].size() == 2);
    REQUIRE(segments[0].contains(0));
    REQUIRE(segments[0].contains(1));
}

TEST_CASE("Lines intersect in the middle and at an endpoint", "[SweepLineIntersectionCalculator]") {
    const std::vector<LineSegment2D> lines{{{0, 0}, {4, 4}}, {{2, 2}, {2, -4}}, {{1, -4}, {5, -4}}};
    SweepLineIntersectionCalculator calculator{};
    const auto result = calculator.FindIntersections(lines);
    const auto& intersections = result.Intersections;
    const auto& segments = result.IntersectingSegments;
    const auto argSort = ArgSort(intersections, VertexAboveComparator{});

    REQUIRE(intersections.size() == 2);
    REQUIRE(AreEqual(intersections[argSort[0]], {2, 2}));
    REQUIRE(AreEqual(intersections[argSort[1]], {2, -4}));

    REQUIRE(segments.size() == 2);
    REQUIRE(segments[argSort[0]].contains(0));
    REQUIRE(segments[argSort[0]].contains(1));
    REQUIRE(segments[argSort[1]].contains(1));
    REQUIRE(segments[argSort[1]].contains(2));
}

TEST_CASE("Lines intersect at their endpoints", "[SweepLineIntersectionCalculator]") {
    const std::vector<LineSegment2D> lines{{{0, 0}, {4, 4}}, {{4, 4}, {6, 5}}, {{6, 5}, {50, 40}}, {{-1, -4}, {0, 0}}};
    SweepLineIntersectionCalculator calculator{};
    const auto result = calculator.FindIntersections(lines);
    const auto& intersections = result.Intersections;
    const auto& segments = result.IntersectingSegments;
    const auto argSort = ArgSort(intersections, VertexAboveComparator{});

    REQUIRE(intersections.size() == 3);
    REQUIRE(AreEqual(intersections[argSort[0]], {6, 5}));
    REQUIRE(AreEqual(intersections[argSort[1]], {4, 4}));
    REQUIRE(AreEqual(intersections[argSort[2]], {0, 0}));

    REQUIRE(segments.size() == 3);
    REQUIRE(segments[argSort[0]].contains(1));
    REQUIRE(segments[argSort[0]].contains(2));
    REQUIRE(segments[argSort[1]].contains(0));
    REQUIRE(segments[argSort[1]].contains(1));
    REQUIRE(segments[argSort[2]].contains(0));
    REQUIRE(segments[argSort[2]].contains(3));
}

TEST_CASE("Two endpoints intersect in the center of a line segment", "[SweepLineIntersectionCalculator]") {
    const std::vector<LineSegment2D> lines{{{0, 4}, {4, 0}}, {{2, 2}, {4, 10}}, {{3, 3}, {2, 2}}};
    SweepLineIntersectionCalculator calculator{};
    const auto result = calculator.FindIntersections(lines);
    const auto& intersections = result.Intersections;
    const auto& segments = result.IntersectingSegments;

    REQUIRE(intersections.size() == 1);
    REQUIRE(AreEqual(intersections[0], {2, 2}));
    REQUIRE(segments.size() == 1);
    REQUIRE(segments[0].size() == 3);
    REQUIRE(segments[0].contains(0));
    REQUIRE(segments[0].contains(1));
    REQUIRE(segments[0].contains(2));
}

TEST_CASE("Four lines intersect at their endpoints", "[SweepLineIntersectionCalculator]") {
    const std::vector<LineSegment2D> lines{{{0, 0}, {2, 1}}, {{1, -2}, {0, 0}}, {{0, 0}, {-2, -1}}, {{0, 0}, {-1, 2}}};
    SweepLineIntersectionCalculator calculator{};
    const auto result = calculator.FindIntersections(lines);
    const auto& intersections = result.Intersections;

    REQUIRE(intersections.size() == 1);
    REQUIRE(AreEqual(intersections[0], {0, 0}));
}

TEST_CASE("Line segments intersect after event point is handled", "[SweepLineIntersectionCalculator]") {
    const std::vector<LineSegment2D> lines{{{0, 0}, {9, 9}}, {{0, 10}, {10, 0}}, {{5, 6}, {4, 15}}};
    SweepLineIntersectionCalculator calculator{};
    const auto result = calculator.FindIntersections(lines);
    const auto& intersections = result.Intersections;

    REQUIRE(intersections.size() == 1);
    REQUIRE(AreEqual(intersections[0], {5, 5}));
}

TEST_CASE("An intersection point has multiple left and right neighbors", "[SweepLineIntersectionCalculator]") {
    const std::vector<LineSegment2D>
        lines{{{0, 0}, {9, 9}}, {{0, 10}, {10, 0}}, {{5, 6}, {4, 15}}, {{-4, 0}, {-9, 100}}, {{15, -10}, {16, 10}}};
    SweepLineIntersectionCalculator calculator{};
    const auto result = calculator.FindIntersections(lines);
    const auto& intersections = result.Intersections;

    REQUIRE(intersections.size() == 1);
    REQUIRE(AreEqual(intersections[0], {5, 5}));
}

TEST_CASE("Multiple lines intersect at the same point") {
    const std::vector<LineSegment2D> lines{{{5, -4}, {2, 8}}, {{4, 0}, {3, -6}},   {{1, -5}, {7, 5}}, {{1, 5}, {4, 0}},
                                           {{4, 0}, {1, 1}},  {{-1, -2}, {-2, 4}}, {{9, 2}, {7, -2}}};
    SweepLineIntersectionCalculator calculator{};
    const auto result = Collapse(calculator.FindIntersections(lines));
    const auto& intersections = result.Intersections;
    const auto& segments = result.IntersectingSegments;

    REQUIRE(intersections.size() == 1);
    REQUIRE(AreEqual(intersections.at(0), {4, 0}));
    REQUIRE(segments.size() == 1);
    REQUIRE(segments.at(0).size() == 5);
    for (const auto i : std::views::iota(0UZ, 5UZ)) {
        REQUIRE(segments.at(0).contains(i));
    }
}