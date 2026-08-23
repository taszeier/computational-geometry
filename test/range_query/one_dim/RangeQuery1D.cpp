#include <catch2/catch_test_macros.hpp>

#include "range_query/one_dim/Range1D.hpp"
#include "range_query/one_dim/RangeQuery1D.hpp"

using namespace compg;

TEST_CASE("RangeQuery1D - Query", "[RangeQuery1D]") {
    BinarySearchTree<int> tree;
    tree.Insert(3);
    tree.Insert(8);
    tree.Insert(7);
    RangeQuery1D query{};

    SECTION("Vertex in on the boundary of the query range") {
        auto queryResult = query.Query(tree, Range1D{1, 3});
        REQUIRE(queryResult.size() == 1);
        REQUIRE(queryResult[0] == 3);
    }

    SECTION("Vertices outside the range are not reported") {
        tree.Insert({-1, 2, 10, 11, 1043});
        auto queryResult = query.Query(tree, Range1D{3, 9});
        std::ranges::sort(queryResult);

        REQUIRE(queryResult.size() == 3);
        REQUIRE(queryResult[0] == 3);
        REQUIRE(queryResult[1] == 7);
        REQUIRE(queryResult[2] == 8);
    }

    SECTION("Query range covers all vertices") {
        auto queryResult = query.Query(tree, Range1D{1, 9});
        std::ranges::sort(queryResult);

        REQUIRE(queryResult.size() == 3);
        REQUIRE(queryResult[0] == 3);
        REQUIRE(queryResult[1] == 7);
        REQUIRE(queryResult[2] == 8);
    }

    SECTION("Query region is to the right of the vertices") {
        auto queryResult = query.Query(tree, Range1D{10, 43});
        REQUIRE(queryResult.empty());
    }

    SECTION("Query region is to the left of the vertices") {
        auto queryResult = query.Query(tree, Range1D{1, 2});
        REQUIRE(queryResult.empty());
    }

    SECTION("Query region does not contain any vertices in the middle") {
        auto queryResult = query.Query(tree, Range1D{4, 5});
        REQUIRE(queryResult.empty());
    }
}
