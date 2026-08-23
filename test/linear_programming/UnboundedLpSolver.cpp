#include <catch2/catch_test_macros.hpp>

#include "linear_programming/UnboundedLpSolver.hpp"

using namespace compg;

TEST_CASE("The solution is unbounded", "[UnboundedLpSolver]") {
    SECTION("The ray is `bounded` to the left") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>>
            halfPlanes{{{{0, 0}, {1, -1}}}, {{{0, 2}, {1, -3}}}, {{{10, 0}, {-1, -1}}}, {{{10, 0}, {2, 1}}}};

        UnboundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, {3, -1});

        const Unbounded expected{{1, -1}};

        REQUIRE(solution.index() == 2);
        REQUIRE(AreEqual(std::get<2>(solution), expected));
    }

    SECTION("The ray is `bounded` to the right") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>>
            halfPlanes{{{{0, 0}, {1, -1}}}, {{{0, 2}, {1, -3}}}, {{{10, 0}, {-1, -1}}}, {{{10, 0}, {2, 1}}}};

        UnboundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, {-1, -2});

        const Unbounded expected{{1, -2}};

        REQUIRE(solution.index() == 2);
        REQUIRE(AreEqual(std::get<2>(solution), expected));
    }

    SECTION("The ray is not `bounded`") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{{{{0, 0}, {1, -1}}}, {{{0, 0}, {1, 2}}}};

        UnboundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, {4, 1});

        const Unbounded expected{{4, 1}};

        REQUIRE(solution.index() == 2);
        REQUIRE(AreEqual(std::get<2>(solution), expected));
    }
}

TEST_CASE("The LP is infeasible") {
    SECTION("The LP with half planes orthogonal to the ray is infeasible") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{{{{0, 1}, {0, 1}}}, {{{0, -1}, {0, -1}}}};

        UnboundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, {1, 0});

        REQUIRE(solution.index() == 0);
    }
}

TEST_CASE("The LP has a `ceiling`") {
    SECTION("The LP has a unique solution and no left intersector") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{{{{1, 0}, {-1, 1}}}, {{{0, 0}, {1, 0}}}};

        UnboundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, {1, -1});

        const Bounded expected{{0, -1}};

        REQUIRE(solution.index() == 1);
        REQUIRE(AreEqual(std::get<1>(solution), expected));
    }

    SECTION("The LP has no unique solution and no left intersector") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{{{{0, 4}, {1, -2}}}, {{{0, 4}, {-1, -1}}}};

        UnboundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, {-1, 2});

        const Unbounded expected{{-2, -1}};

        REQUIRE(solution.index() == 2);
        REQUIRE(AreEqual(std::get<2>(solution), expected));
    }

    SECTION("The LP is infeasible and has no left intersector") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{
            {{{0, 4}, {1, -2}}},
            {{{0, 4}, {-1, -1}}},
            {{{0, 6}, {-1, 2}}}
        };

        UnboundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, {-1, 2});

        REQUIRE(solution.index() == 0);
    }

    SECTION("The ceiling is the only half plane") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{{{{0, 4}, {1, -2}}}};

        UnboundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, {-1, 2});

        const Unbounded expected{{-2, -1}};
        REQUIRE(solution.index() == 2);
        REQUIRE(AreEqual(std::get<2>(solution), expected));
    }

    SECTION("The LP has a unique solution and a left intersector") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{{{{0, 4}, {1, -2}}}, {{{0, 4}, {1, 1}}}};

        UnboundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, {-1, 2});

        const Bounded expected{{0, 4}};

        REQUIRE(solution.index() == 1);
        REQUIRE(AreEqual(std::get<1>(solution), expected));
    }

    SECTION("The LP is not unbounded") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{
            {{{0, 4}, {1, -2}}},
            {{{0, 4}, {1, 1}}},
            {{{0, 2}, {0, -1}}}
        };

        UnboundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, {-1, 2});

        const Bounded expected{{2, 2}};

        REQUIRE(solution.index() == 1);
        REQUIRE(AreEqual(std::get<1>(solution), expected));
    }
}

TEST_CASE("The solution is bounded", "[UnboundedLpSolver]") {
    SECTION("The feasible region is unbounded") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{{{{0, 0}, {1, 2}}}, {{{0, 10}, {-1, -1}}}};

        UnboundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, {0, -1});

        const Bounded expected{{20, -10}};

        REQUIRE(solution.index() == 1);
        REQUIRE(AreEqual(std::get<1>(solution), expected));
    }

    SECTION("The feasible region is bounded") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{
            {{{0, 0}, {1, 2}}},
            {{{0, 10}, {-1, -1}}},
            {{{0, 0}, {1, -1}}}
        };

        UnboundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, {-3, -1});

        const Bounded expected{{0, 0}};

        REQUIRE(solution.index() == 1);
        REQUIRE(AreEqual(std::get<1>(solution), expected));
    }

    SECTION("There are multiple feasible points that maximize the cost") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>>
            halfPlanes{{{{0, -2}, {1, 1}}}, {{{0, 2}, {1, -1}}}, {{{0, -2}, {-1, 1}}}, {{{0, 2}, {-1, -1}}}};

        UnboundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, {1, 1});

        const Bounded expected{{0, 2}};

        REQUIRE(solution.index() == 1);
        REQUIRE(AreEqual(std::get<1>(solution), expected));
    }
}

TEST_CASE("The cost function is zero") {

    SECTION("There is a unique solution in the feasible region") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{
            {{{0, 0}, {1, 2}}},
            {{{0, 10}, {-1, -1}}},
            {{{0, 0}, {1, -1}}}
        };

        UnboundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, {0, 0});

        const Bounded expected{{0, 0}};

        REQUIRE(solution.index() == 1);
        REQUIRE(AreEqual(std::get<1>(solution), expected));
    }

    SECTION("There are multiple solutions in the feasible region") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>>
            halfPlanes{{{{0, 0}, {1, 2}}}, {{{0, 10}, {-1, -1}}}, {{{0, 0}, {1, -1}}}, {{{2, 0}, {1, 0}}}};

        UnboundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, {0, 0});

        const Bounded expected{{2, -1}};

        REQUIRE(solution.index() == 1);
        REQUIRE(AreEqual(std::get<1>(solution), expected));
    }

    SECTION("There is a unique unbounded solution") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{{{{0, 0}, {1, 0}}}};

        UnboundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, {0, 0});

        const Unbounded expected{{0, -1}};

        REQUIRE(solution.index() == 2);
        REQUIRE(AreEqual(std::get<2>(solution), expected));
    }

    SECTION("There is no unique unbounded solution") {
        const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes{{{{0, 0}, {-1, -1}}}, {{{0, 0}, {-1, 1}}}};

        UnboundedLpSolver solver{};
        const auto solution = solver.Solve(halfPlanes, {0, 0});

        const Unbounded expected{{-1, 0}};

        REQUIRE(solution.index() == 2);
        REQUIRE(AreEqual(std::get<2>(solution), expected));
    }
}

TEST_CASE("Zero half planes") {
    const std::vector<HalfPlane<Hyperplane<2UZ>>> halfPlanes;

    UnboundedLpSolver solver{};
    const auto solution = solver.Solve(halfPlanes, {0, 1});

    const Unbounded expected{{0, 1}};

    REQUIRE(solution.index() == 2);
    REQUIRE(AreEqual(std::get<2>(solution), expected));
}