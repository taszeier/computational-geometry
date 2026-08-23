#include <catch2/catch_test_macros.hpp>

#include "common/Vertex.hpp"
#include "math/Math.hpp"
#include "range_query/kd_tree/BallQueryRegion.hpp"
#include "range_query/kd_tree/KdTree.hpp"
#include "range_query/kd_tree/KdTreeQuery.hpp"
#include "range_query/kd_tree/OrthogonalQueryRegion.hpp"

using namespace compg;

TEST_CASE(
    "KdTreeQuery - vertices collinear to orthogonal range query lines are "
    "excluded",
    "[KdTreeQuery]"
) {
    std::vector<Vertex2D> vertices;
    vertices.reserve(9);
    std::ranges::for_each(std::views::cartesian_product(Indices<3>, Indices<3>), [&vertices](const auto indices) {
        const auto [i, j] = indices;
        vertices.push_back(Vertex2D{i, j});
    });
    KdTree<2> tree{vertices};
    KdTreeQuery<2> query;

    SECTION("Query region covers bottom left corner") {
        auto queryResult = query.Query(tree, OrthogonalQueryRegion<2>{{{0, 0}, {1, 1}}});

        std::ranges::sort(queryResult, LexicographicalLess<Vertex<2>>{}, ValueProjector{});
        REQUIRE(queryResult.size() == 4);
        REQUIRE(AreEqual(queryResult.at(0).Value, {0, 0}));
        REQUIRE(queryResult.at(0).Index == 0);
        REQUIRE(AreEqual(queryResult.at(1).Value, {0, 1}));
        REQUIRE(queryResult.at(1).Index == 1);
        REQUIRE(AreEqual(queryResult.at(2).Value, {1, 0}));
        REQUIRE(queryResult.at(2).Index == 3);
        REQUIRE(AreEqual(queryResult.at(3).Value, {1, 1}));
        REQUIRE(queryResult.at(3).Index == 4);
    }

    SECTION("Query region covers top left corner") {
        auto queryResult = query.Query(tree, OrthogonalQueryRegion<2>{{{0, 1}, {1, 2}}});

        std::ranges::sort(queryResult, LexicographicalLess<Vertex<2>>{}, ValueProjector{});
        REQUIRE(queryResult.size() == 4);
        REQUIRE(AreEqual(queryResult.at(0).Value, {0, 1}));
        REQUIRE(queryResult.at(0).Index == 1);
        REQUIRE(AreEqual(queryResult.at(1).Value, {0, 2}));
        REQUIRE(queryResult.at(1).Index == 2);
        REQUIRE(AreEqual(queryResult.at(2).Value, {1, 1}));
        REQUIRE(queryResult.at(2).Index == 4);
        REQUIRE(AreEqual(queryResult.at(3).Value, {1, 2}));
        REQUIRE(queryResult.at(3).Index == 5);
    }

    SECTION("Query region covers top right corner") {
        auto queryResult = query.Query(tree, OrthogonalQueryRegion<2>{{{1, 1}, {2, 2}}});

        std::ranges::sort(queryResult, LexicographicalLess<Vertex<2>>{}, ValueProjector{});
        REQUIRE(queryResult.size() == 4);
        REQUIRE(AreEqual(queryResult.at(0).Value, {1, 1}));
        REQUIRE(queryResult.at(0).Index == 4);
        REQUIRE(AreEqual(queryResult.at(1).Value, {1, 2}));
        REQUIRE(queryResult.at(1).Index == 5);
        REQUIRE(AreEqual(queryResult.at(2).Value, {2, 1}));
        REQUIRE(queryResult.at(2).Index == 7);
        REQUIRE(AreEqual(queryResult.at(3).Value, {2, 2}));
        REQUIRE(queryResult.at(3).Index == 8);
    }

    SECTION("Query region covers bottom right corner") {
        auto queryResult = query.Query(tree, OrthogonalQueryRegion<2>{{{1, 0}, {2, 1}}});

        std::ranges::sort(queryResult, LexicographicalLess<Vertex<2>>{}, ValueProjector{});
        REQUIRE(queryResult.size() == 4);
        REQUIRE(AreEqual(queryResult.at(0).Value, {1, 0}));
        REQUIRE(queryResult.at(0).Index == 3);
        REQUIRE(AreEqual(queryResult.at(1).Value, {1, 1}));
        REQUIRE(queryResult.at(1).Index == 4);
        REQUIRE(AreEqual(queryResult.at(2).Value, {2, 0}));
        REQUIRE(queryResult.at(2).Index == 6);
        REQUIRE(AreEqual(queryResult.at(3).Value, {2, 1}));
        REQUIRE(queryResult.at(3).Index == 7);
    }
}

TEST_CASE("KdTreeQuery - query region covers all vertices", "[KdTreeQuery]") {
    const std::vector<Vertex2D> vertices{{1, 1}, {1, 2}, {1.5, 1.5}};
    KdTree<2> tree{vertices};
    KdTreeQuery<2> query;

    SECTION("Orthogonal query region covers all vertices") {
        auto queryResult = query.Query(tree, OrthogonalQueryRegion<2>{{{0, 0}, {2, 2}}});
        std::ranges::sort(queryResult, LexicographicalLess<Vertex2D>{}, ValueProjector{});

        REQUIRE(queryResult.size() == 3);
        REQUIRE(AreEqual(queryResult.at(0).Value, {1, 1}));
        REQUIRE(queryResult.at(0).Index == 0);
        REQUIRE(AreEqual(queryResult.at(1).Value, {1, 2}));
        REQUIRE(queryResult.at(1).Index == 1);
        REQUIRE(AreEqual(queryResult.at(2).Value, {1.5, 1.5}));
        REQUIRE(queryResult.at(2).Index == 2);
    }

    SECTION("Ball query region covers all vertices") {
        auto queryResult = query.Query(tree, BallQueryRegion<2>{{{1, 1}, 1}});
        std::ranges::sort(queryResult, LexicographicalLess<Vertex2D>{}, ValueProjector{});

        REQUIRE(queryResult.size() == 3);
        REQUIRE(AreEqual(queryResult.at(0).Value, {1, 1}));
        REQUIRE(queryResult.at(0).Index == 0);
        REQUIRE(AreEqual(queryResult.at(1).Value, {1, 2}));
        REQUIRE(queryResult.at(1).Index == 1);
        REQUIRE(AreEqual(queryResult.at(2).Value, {1.5, 1.5}));
        REQUIRE(queryResult.at(2).Index == 2);
    }
}

TEST_CASE("KdTreeQuery - query region does not cover any vertices", "[KdTreeQuery]") {
    const std::vector<Vertex2D> vertices{{1, 1}, {3, 3}};
    KdTree<2> tree{vertices};
    KdTreeQuery<2> query;

    SECTION("Orthogonal query region covers zero vertices") {
        auto queryResult = query.Query(tree, OrthogonalQueryRegion<2>{{{1.5, 1.5}, {2.5, 2.5}}});

        REQUIRE(queryResult.empty());
    }

    SECTION("Ball query region covers zero vertices") {
        auto queryResult = query.Query(tree, BallQueryRegion<2>{{{2, 2}, 0.5}});

        REQUIRE(queryResult.empty());
    }
}