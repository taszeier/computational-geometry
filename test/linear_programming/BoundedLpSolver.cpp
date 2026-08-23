#include <catch2/catch_test_macros.hpp>

#include "linear_programming/BoundedLpSolver.hpp"

using namespace compg;

TEST_CASE("Zero half planes", "[BoundedLpSolver]") {
    SECTION("cost[0] > 0 and cost[1] > 0") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{};
        BoundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, 10, {1, 1});

        const Bounded expected{{10, 10}};

        REQUIRE(solution.index() == 1);
        REQUIRE(AreEqual(std::get<1>(solution), expected));
    }

    SECTION("The cost is zero") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{};
        BoundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, 10, {0, 0});

        const Bounded expected{{-10, -10}};

        REQUIRE(solution.index() == 1);
        REQUIRE(AreEqual(std::get<1>(solution), expected));
    }

    SECTION("cost[0] < 0 and cost[1] > 0") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{};
        BoundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, 10, {-1, 1});

        const Bounded expected{{-10, 10}};

        REQUIRE(solution.index() == 1);
        REQUIRE(AreEqual(std::get<1>(solution), expected));
    }
}

TEST_CASE("The LP is infeasible", "[BoundedLpSolver]") {
    const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{{{{0, 2}, {-1, 1}}}, {{{2, 0}, {1, 0}}}};
    BoundedLpSolver solver{};
    const auto solution = solver.Solve(halfPlanes, 3, {1, 1});

    REQUIRE(solution.index() == 0);
}

TEST_CASE("The LP is feasible", "[BoundedLpSolver]") {
    SECTION("There are multiple points that maximize the cost") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{{{{0, 2}, {1, -1}}}, {{{2, 0}, {1, 0}}}};
        BoundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, 3, {-1, 0});

        const Bounded expected{{2, -3}};

        REQUIRE(solution.index() == 1);
        REQUIRE(AreEqual(std::get<1>(solution), expected));
    }

    SECTION("There is a unique point that maximizes the cost") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{{{{0, 2}, {1, -1}}}, {{{2, 0}, {-1, 0}}}};
        BoundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, 10, {0, 1});

        const Bounded expected{{2, 4}};

        REQUIRE(solution.index() == 1);
        REQUIRE(AreEqual(std::get<1>(solution), expected));
    }

    SECTION("The cost is zero") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{
            {{{0, 0}, {1, -1}}},
            {{{0, 0}, {1, 2}}},
            {{{0, 10}, {-1, -1}}},
        };
        BoundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, 100, {0, 0});

        const Bounded expected{{0, 0}};

        REQUIRE(solution.index() == 1);
        REQUIRE(AreEqual(std::get<1>(solution), expected));
    }
}