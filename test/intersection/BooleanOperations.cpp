#include <catch2/catch_test_macros.hpp>

#include "data_structures//DoublyConnectedEdgeListAlgorithms.hpp"
#include "intersection/BooleanOperations.hpp"
#include "math/Conversions.hpp"

using namespace compg;

TEST_CASE("Boolean operations", "[BooleanOperations]") {
    auto edgeList1 = ConvertTo<DoublyConnectedEdgeList>(Polygon{{{0, 0}, {3, 3}, {0, 6}}});
    UpdateFaces(edgeList1);

    auto edgeList2 = ConvertTo<DoublyConnectedEdgeList>(Polygon{{{1, 3}, {4, 0}, {4, 6}}});
    UpdateFaces(edgeList2);

    SECTION("Union") {
        const auto result = Union(edgeList1, edgeList2);
        DoublyConnectedEdgeList expected;
        {
            const auto v0 = expected.InsertVertex({0, 0});
            const auto v1 = expected.InsertVertex({2, 2});
            const auto v2 = expected.InsertVertex({1, 3});
            const auto v3 = expected.InsertVertex({2, 4});
            const auto v4 = expected.InsertVertex({0, 6});

            const auto v5 = expected.InsertVertex({3, 3});
            const auto v6 = expected.InsertVertex({4, 0});
            const auto v7 = expected.InsertVertex({4, 6});

            expected.InsertEdge(v0, v1);
            expected.InsertEdge(v1, v2);
            expected.InsertEdge(v2, v3);
            expected.InsertEdge(v3, v4);
            expected.InsertEdge(v4, v0);

            expected.InsertEdge(v1, v5);
            expected.InsertEdge(v3, v5);
            expected.InsertEdge(v1, v6);
            expected.InsertEdge(v3, v7);
            expected.InsertEdge(v6, v7);
        }

        CHECK(AreIsomorphic(result, expected));
    }

    SECTION("Intersection") {
        const auto result = Intersection(edgeList1, edgeList2);
        const auto expected = ConvertTo<DoublyConnectedEdgeList>(Polygon{{{1, 3}, {2, 2}, {3, 3}, {2, 4}}});

        CHECK(AreIsomorphic(result, expected));
    }

    SECTION("Difference") {
        const auto result = Difference(edgeList1, edgeList2);
        const auto expected = ConvertTo<DoublyConnectedEdgeList>(Polygon{{{0, 0}, {2, 2}, {1, 3}, {2, 4}, {0, 6}}});

        CHECK(AreIsomorphic(result, expected));
    }
}