#include <catch2/catch_test_macros.hpp>

#include <ranges>

#include "common/Common.hpp"
#include "math/Math.hpp"
#include "math/primitives/UnboundedBox.hpp"

using namespace compg;

TEST_CASE("UnboundedBox - IsBounded", "[UnboundedBox]") {
    UnboundedBox<2> box;
    box.Intersect(AxesAlignedHyperplane<2>{0, -1}, Side::Positive);
    box.Intersect(AxesAlignedHyperplane<2>{0, 1}, Side::Negative);
    box.Intersect(AxesAlignedHyperplane<2>{1, -1}, Side::Positive);

    SECTION("Box with three sides is unbounded") {
        REQUIRE_FALSE(box.IsBounded());
    }

    SECTION("Non-empty box with four sides in bounded") {
        box.Intersect(AxesAlignedHyperplane<2>{1, 1}, Side::Negative);
        REQUIRE(box.IsBounded());
    }

    SECTION("Empty box with three side is bounded") {
        box.Intersect(AxesAlignedHyperplane<2>{0, -10}, Side::Negative);
        REQUIRE(box.IsBounded());
    }

    SECTION("Half line box is not bounded") {
        box.Intersect(AxesAlignedHyperplane<2>{0, -1}, Side::Negative);
        REQUIRE_FALSE(box.IsBounded());
    }

    SECTION("Box of the entire plane is unbounded") {
        const UnboundedBox<2> planeBox;
        REQUIRE_FALSE(planeBox.IsBounded());
    }
}

TEST_CASE("UnboundedBox - IsEmpty", "[UnboundedBox]") {
    UnboundedBox<2> box;
    box.Intersect(AxesAlignedHyperplane<2>{0, -1}, Side::Positive);
    box.Intersect(AxesAlignedHyperplane<2>{0, 1}, Side::Negative);

    SECTION("Unbounded box is not empty") {
        REQUIRE_FALSE(box.IsEmpty());
    }

    SECTION("Non-empty bounded box is not empty") {
        box.Intersect(AxesAlignedHyperplane<2>{1, -1}, Side::Positive);
        box.Intersect(AxesAlignedHyperplane<2>{1, 1}, Side::Negative);
        REQUIRE_FALSE(box.IsEmpty());
    }

    SECTION("Empty box with two sides is empty") {
        box.Intersect(AxesAlignedHyperplane<2>{0, -10}, Side::Negative);
        REQUIRE(box.IsEmpty());
    }

    SECTION("Box of the entire plane is not empty") {
        const UnboundedBox<2> planeBox;
        REQUIRE_FALSE(planeBox.IsEmpty());
    }
}

TEST_CASE("UnboundedBox - GetLowerCorner", "[UnboundedBox]") {
    SECTION("Box of entire plane does not have a lower corner") {
        const UnboundedBox<2> box;
        REQUIRE_FALSE(box.GetLowerCorner().has_value());
    }

    SECTION("Box with only one lower bound does not necessarilyhave a lower corner") {
        UnboundedBox<2> box;
        box.Intersect(AxesAlignedHyperplane<2>{1, 0}, Side::Positive);
        REQUIRE_FALSE(box.GetLowerCorner().has_value());
    }

    SECTION("Box with upper corner does not have lower corner") {
        UnboundedBox<2> box;
        box.Intersect(AxesAlignedHyperplane<2>{0, 0}, Side::Negative);
        box.Intersect(AxesAlignedHyperplane<2>{1, 0}, Side::Negative);
        REQUIRE_FALSE(box.GetLowerCorner().has_value());
    }

    SECTION("Box with both lower bounds has a lower corner") {
        UnboundedBox<2> box;
        box.Intersect(AxesAlignedHyperplane<2>{1, 0}, Side::Positive);
        box.Intersect(AxesAlignedHyperplane<2>{0, 0}, Side::Positive);
        const auto lowerCorner = box.GetLowerCorner();

        REQUIRE(lowerCorner.has_value());
        REQUIRE(AreEqual(lowerCorner.value(), {0, 0}));
    }

    SECTION("Non-empty bounded box has lower corner") {
        UnboundedBox<2> box;
        box.Intersect(AxesAlignedHyperplane<2>{1, 0}, Side::Positive);
        box.Intersect(AxesAlignedHyperplane<2>{0, 0}, Side::Positive);
        box.Intersect(AxesAlignedHyperplane<2>{1, 1}, Side::Negative);
        box.Intersect(AxesAlignedHyperplane<2>{0, 1}, Side::Negative);

        const auto lowerCorner = box.GetLowerCorner();
        REQUIRE(lowerCorner.has_value());
        REQUIRE(AreEqual(lowerCorner.value(), {0, 0}));
    }

    SECTION("Empty box does not have a lower corner") {
        UnboundedBox<2> box;
        box.Intersect(AxesAlignedHyperplane<2>{1, 0}, Side::Positive);
        box.Intersect(AxesAlignedHyperplane<2>{0, 0}, Side::Positive);
        box.Intersect(AxesAlignedHyperplane<2>{1, 1}, Side::Negative);
        box.Intersect(AxesAlignedHyperplane<2>{0, 1}, Side::Negative);
        box.Intersect(AxesAlignedHyperplane<2>{0, 10}, Side::Positive);

        REQUIRE_FALSE(box.GetLowerCorner().has_value());
    }
}

TEST_CASE("UnboundedBox - GetUpperCorner", "[UnboundedBox]") {
    SECTION("Box of entire plane does not have an upper corner") {
        const UnboundedBox<2> box;
        REQUIRE_FALSE(box.GetUpperCorner().has_value());
    }

    SECTION("Box with only one upper bound does not have an upper corner") {
        UnboundedBox<2> box;
        box.Intersect(AxesAlignedHyperplane<2>{1, 0}, Side::Negative);
        REQUIRE_FALSE(box.GetUpperCorner().has_value());
    }

    SECTION("Box with lower corner does not necessarily have upper corner") {
        UnboundedBox<2> box;
        box.Intersect(AxesAlignedHyperplane<2>{0, 0}, Side::Positive);
        box.Intersect(AxesAlignedHyperplane<2>{1, 0}, Side::Positive);
        REQUIRE_FALSE(box.GetUpperCorner().has_value());
    }

    SECTION("Box with both upper bounds has an upper corner") {
        UnboundedBox<2> box;
        box.Intersect(AxesAlignedHyperplane<2>{1, 0}, Side::Negative);
        box.Intersect(AxesAlignedHyperplane<2>{0, 0}, Side::Negative);
        const auto upperCorner = box.GetUpperCorner();

        REQUIRE(upperCorner.has_value());
        REQUIRE(AreEqual(upperCorner.value(), {0, 0}));
    }

    SECTION("Non-empty bounded box has upper corner") {
        UnboundedBox<2> box;
        box.Intersect(AxesAlignedHyperplane<2>{1, 0}, Side::Positive);
        box.Intersect(AxesAlignedHyperplane<2>{0, 0}, Side::Positive);
        box.Intersect(AxesAlignedHyperplane<2>{1, 1}, Side::Negative);
        box.Intersect(AxesAlignedHyperplane<2>{0, 1}, Side::Negative);

        const auto upperCorner = box.GetUpperCorner();
        REQUIRE(upperCorner.has_value());
        REQUIRE(AreEqual(upperCorner.value(), {1, 1}));
    }

    SECTION("Empty box does not have an upper corner") {
        UnboundedBox<2> box;
        box.Intersect(AxesAlignedHyperplane<2>{1, 0}, Side::Positive);
        box.Intersect(AxesAlignedHyperplane<2>{0, 0}, Side::Positive);
        box.Intersect(AxesAlignedHyperplane<2>{1, 1}, Side::Negative);
        box.Intersect(AxesAlignedHyperplane<2>{0, 1}, Side::Negative);
        box.Intersect(AxesAlignedHyperplane<2>{0, 10}, Side::Positive);

        REQUIRE_FALSE(box.GetUpperCorner().has_value());
    }
}

TEST_CASE("UnboundedBox - GetCorners", "[UnboundedBox]") {
    SECTION("Box of the entire plane does not have any corners") {
        const UnboundedBox<2> box;
        auto maybeCorners = box.GetCorners();
        REQUIRE(maybeCorners.has_value());

        auto corners = maybeCorners.value() | std::ranges::to<std::vector>();
        REQUIRE(corners.empty());
    }

    UnboundedBox<2> box;
    box.Intersect(AxesAlignedHyperplane<2>{0, -1}, Side::Positive);
    box.Intersect(AxesAlignedHyperplane<2>{0, 1}, Side::Negative);

    SECTION("Box with two opposite side does not have any corners") {
        auto maybeCorners = box.GetCorners();
        REQUIRE(maybeCorners.has_value());

        auto corners = maybeCorners.value() | std::ranges::to<std::vector>();
        REQUIRE(corners.empty());
    }

    SECTION("Box with three sides has two corners") {
        box.Intersect(AxesAlignedHyperplane<2>{1, -1}, Side::Positive);
        auto maybeCorners = box.GetCorners();
        REQUIRE(maybeCorners.has_value());

        auto corners = maybeCorners.value() | std::ranges::to<std::vector>();
        std::ranges::sort(corners, LexicographicalLess<Vertex2D>{});
        REQUIRE(corners.size() == 2);
        REQUIRE(AreEqual(corners[0], {-1, -1}));
        REQUIRE(AreEqual(corners[1], {1, -1}));
    }

    SECTION("Box with all four sides has four corners") {
        box.Intersect(AxesAlignedHyperplane<2>{1, -1}, Side::Positive);
        box.Intersect(AxesAlignedHyperplane<2>{1, 1}, Side::Negative);
        auto maybeCorners = box.GetCorners();
        REQUIRE(maybeCorners.has_value());

        auto corners = maybeCorners.value() | std::ranges::to<std::vector>();
        std::ranges::sort(corners, LexicographicalLess<Vertex2D>{});
        REQUIRE(corners.size() == 4);
        REQUIRE(AreEqual(corners[0], {-1, -1}));
        REQUIRE(AreEqual(corners[1], {-1, 1}));
        REQUIRE(AreEqual(corners[2], {1, -1}));
        REQUIRE(AreEqual(corners[3], {1, 1}));
    }

    SECTION("Empty box does not have any corners") {
        box.Intersect(AxesAlignedHyperplane<2>{0, -10}, Side::Negative);
        REQUIRE_FALSE(box.GetCorners().has_value());
    }
}

TEST_CASE("UnboundedBox - GetUpperPlanes", "[UnboundedBox]") {
    SECTION("Box of entire plane does not have any upper planes") {
        const UnboundedBox<2> box;
        auto maybeUpperPlanes = box.GetUpperPlanes();
        REQUIRE(maybeUpperPlanes.has_value());

        auto upperPlanes = maybeUpperPlanes.value() | std::ranges::to<std::vector>();
        REQUIRE(upperPlanes.empty());
    }

    SECTION("Box with only one upper bound only has one upper plane") {
        UnboundedBox<2> box;
        box.Intersect(AxesAlignedHyperplane<2>{1, 0}, Side::Negative);

        auto maybeUpperPlanes = box.GetUpperPlanes();
        REQUIRE(maybeUpperPlanes.has_value());

        const auto upperPlanes = maybeUpperPlanes.value() | std::ranges::to<std::vector>();
        REQUIRE(upperPlanes.size() == 1);
        REQUIRE(upperPlanes[0].GetAxisIndex() == 1);
        REQUIRE(upperPlanes[0].GetIntersection() == 0);
    }

    SECTION("Box with lower corner does not necessarily have any upper planes") {
        UnboundedBox<2> box;
        box.Intersect(AxesAlignedHyperplane<2>{0, 0}, Side::Positive);
        box.Intersect(AxesAlignedHyperplane<2>{1, 0}, Side::Positive);
        auto maybeUpperPlanes = box.GetUpperPlanes();
        REQUIRE(maybeUpperPlanes.has_value());

        auto upperPlanes = maybeUpperPlanes.value() | std::ranges::to<std::vector>();
        REQUIRE(upperPlanes.empty());
    }

    SECTION("Box with both upper bounds has two upper planes") {
        UnboundedBox<2> box;
        box.Intersect(AxesAlignedHyperplane<2>{1, 1}, Side::Negative);
        box.Intersect(AxesAlignedHyperplane<2>{0, 0}, Side::Negative);
        auto maybeUpperPlanes = box.GetUpperPlanes();
        REQUIRE(maybeUpperPlanes.has_value());

        auto upperPlanes = maybeUpperPlanes.value() | std::ranges::to<std::vector>();
        std::ranges::sort(upperPlanes, Less{}, [](const auto& p) { return p.GetAxisIndex(); });
        REQUIRE(upperPlanes.size() == 2);
        REQUIRE(upperPlanes[0].GetAxisIndex() == 0);
        REQUIRE(upperPlanes[0].GetIntersection() == 0);
        REQUIRE(upperPlanes[1].GetAxisIndex() == 1);
        REQUIRE(upperPlanes[1].GetIntersection() == 1);
    }

    SECTION("Empty box does not have any upper planes") {
        UnboundedBox<2> box;
        box.Intersect(AxesAlignedHyperplane<2>{1, 0}, Side::Positive);
        box.Intersect(AxesAlignedHyperplane<2>{0, 0}, Side::Positive);
        box.Intersect(AxesAlignedHyperplane<2>{1, 1}, Side::Negative);
        box.Intersect(AxesAlignedHyperplane<2>{0, 1}, Side::Negative);
        box.Intersect(AxesAlignedHyperplane<2>{0, 10}, Side::Positive);

        REQUIRE_FALSE(box.GetUpperPlanes().has_value());
    }
}
