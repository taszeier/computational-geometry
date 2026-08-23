#include <catch2/catch_test_macros.hpp>

#include "linear_programming/ConvexPolygon.hpp"
#include "linear_programming/HalfPlaneIntersector.hpp"

using namespace compg;

TEST_CASE("as", "[HalfPlaneIntersector]") {
    const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{
        {{{0, 1}, {1, -1}}},
        {{{1, 0}, {-2, -1}}},
        {{{-1, 0}, {1, 2}}}
    };
    HalfPlaneIntersector intersector{};
    const auto intersection = intersector.Intersect(halfPlanes);

    const ConvexPolygon expected{{halfPlanes.at(0).Plane, halfPlanes.at(2).Plane}, {halfPlanes.at(1).Plane}};

    REQUIRE(AreEqual(intersection, expected));
}

TEST_CASE("", "[HalfPlaneIntersector]") {
    const std::vector<HalfPlane<Hyperplane<2UZ>>>
        halfPlanes{{{{-1, 0}, {1, 1}}}, {{{0, 1}, {-1, -1}}}, {{{0, 1}, {1, -1}}}, {{{1, 0}, {-1, 1}}}};

    HalfPlaneIntersector intersector{};
    const auto intersection = intersector.Intersect(halfPlanes);

    const ConvexPolygon expected{
        {halfPlanes.at(2).Plane, halfPlanes.at(0).Plane},
        {halfPlanes.at(1).Plane, halfPlanes.at(3).Plane}
    };

    REQUIRE(AreEqual(intersection, expected));
}

TEST_CASE("No duplicate edges are created when there is a double left-left intersection", "[HalfPlaneIntersector]") {
    const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{
        {{{0, 0}, {1, -1}}},
        {{{0, 0}, {1, 2}}}, // {0, 1}
        {{{2, 0}, {1, 0}}}
    };

    HalfPlaneIntersector intersector{};
    const auto intersection = intersector.Intersect(halfPlanes);

    const ConvexPolygon expected{{halfPlanes.at(0).Plane, halfPlanes.at(2).Plane, halfPlanes.at(1).Plane}, {}};

    REQUIRE(AreEqual(intersection, expected));
}

TEST_CASE("The intersection is a closed polygon", "[HalfPlaneIntersector]") {
    const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{{{{0, 8}, {0, -1}}}, {{{0, 5}, {1, -1}}},
                                                             {{{0, 4}, {1, -2}}}, {{{5, 5}, {-1, -2}}},

                                                             {{{0, 0}, {-1, 1}}}, {{{0, 1}, {0, 1}}},
                                                             {{{0, 0}, {4, 1}}}};

    HalfPlaneIntersector intersector{};
    const auto intersection = intersector.Intersect(halfPlanes);

    const ConvexPolygon expected{
        .LeftBoundary = {halfPlanes.at(2).Plane, halfPlanes.at(6).Plane, halfPlanes.at(5).Plane},
        .RightBoundary = {halfPlanes.at(3).Plane, halfPlanes.at(4).Plane}
    };

    REQUIRE(AreEqual(intersection, expected));
}