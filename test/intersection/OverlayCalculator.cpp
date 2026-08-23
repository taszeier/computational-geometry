#include <catch2/catch_test_macros.hpp>

#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include "intersection/OverlayCalculator.hpp"

using namespace compg;

TEST_CASE("OverlayCalculator - An edge is intersected twice", "[OverlayCalculator]") {
    DoublyConnectedEdgeList edgeList1;
    {
        const auto v0 = edgeList1.InsertVertex({0, 2});
        const auto v1 = edgeList1.InsertVertex({2, 0});
        const auto v2 = edgeList1.InsertVertex({4, 1});
        const auto v3 = edgeList1.InsertVertex({4, 2});
        const auto v4 = edgeList1.InsertVertex({2, 4});

        edgeList1.InsertEdge(v0, v1);
        edgeList1.InsertEdge(v1, v2);
        edgeList1.InsertEdge(v2, v3);
        edgeList1.InsertEdge(v3, v4);
        edgeList1.InsertEdge(v4, v0);
    }
    DoublyConnectedEdgeList edgeList2;
    {
        const auto v0 = edgeList2.InsertVertex({0, 0});
        const auto v1 = edgeList2.InsertVertex({6, 6});
        const auto v2 = edgeList2.InsertVertex({6, -6});

        edgeList2.InsertEdge(v0, v1);
        edgeList2.InsertEdge(v1, v2);
        edgeList2.InsertEdge(v2, v0);
    }
    OverlayCalculator calc{};
    const auto overlay = calc.FindOverlay(edgeList1, edgeList2);

    DoublyConnectedEdgeList expected;
    {
        const auto v0 = expected.InsertVertex({0, 2});
        const auto v1 = expected.InsertVertex({2, 0});
        const auto v2 = expected.InsertVertex({4, 1});
        const auto v3 = expected.InsertVertex({4, 2});
        const auto v4 = expected.InsertVertex({2, 4});

        const auto v5 = expected.InsertVertex({0, 0});
        const auto v6 = expected.InsertVertex({6, 6});
        const auto v7 = expected.InsertVertex({6, -6});

        const auto v8 = expected.InsertVertex({1, 1});
        const auto v9 = expected.InsertVertex({3, 3});

        expected.InsertEdge(v0, v8);
        expected.InsertEdge(v8, v1);
        expected.InsertEdge(v1, v2);
        expected.InsertEdge(v2, v3);
        expected.InsertEdge(v3, v9);
        expected.InsertEdge(v9, v4);
        expected.InsertEdge(v4, v0);
        expected.InsertEdge(v9, v8);
        expected.InsertEdge(v8, v5);
        expected.InsertEdge(v9, v6);
        expected.InsertEdge(v7, v6);
        expected.InsertEdge(v7, v5);
    }

    REQUIRE(AreIsomorphic(expected, overlay));
}

TEST_CASE("OverlayCalculator - Has all three types of intersections", "[OverlayCalculator]") {
    DoublyConnectedEdgeList edgeList1;
    {
        const auto v0 = edgeList1.InsertVertex({0, 2});
        const auto v1 = edgeList1.InsertVertex({2, 0});
        const auto v2 = edgeList1.InsertVertex({4, 2});
        const auto v3 = edgeList1.InsertVertex({2, 4});

        edgeList1.InsertEdge(v0, v1);
        edgeList1.InsertEdge(v1, v2);
        edgeList1.InsertEdge(v2, v3);
        edgeList1.InsertEdge(v3, v0);

        const auto v11 = edgeList1.InsertVertex({2, 7});
        const auto v12 = edgeList1.InsertVertex({4, 5});
        const auto v13 = edgeList1.InsertVertex({4, 7});
        const auto v14 = edgeList1.InsertVertex({3, 8});

        edgeList1.InsertEdge(v11, v12);
        edgeList1.InsertEdge(v12, v13);
        edgeList1.InsertEdge(v13, v14);
        edgeList1.InsertEdge(v14, v11);
    }

    DoublyConnectedEdgeList edgeList2;
    {
        const auto v4 = edgeList2.InsertVertex({2, 2});
        const auto v5 = edgeList2.InsertVertex({4, 0});
        const auto v6 = edgeList2.InsertVertex({6, 2});
        const auto v7 = edgeList2.InsertVertex({4, 4});

        edgeList2.InsertEdge(v4, v5);
        edgeList2.InsertEdge(v5, v6);
        edgeList2.InsertEdge(v6, v7);
        edgeList2.InsertEdge(v7, v4);

        const auto v8 = edgeList2.InsertVertex({2, 4});
        const auto v9 = edgeList2.InsertVertex({0, 4});
        const auto v10 = edgeList2.InsertVertex({3, 6});

        edgeList2.InsertEdge(v8, v9);
        edgeList2.InsertEdge(v9, v10);
        edgeList2.InsertEdge(v10, v8);
    }

    OverlayCalculator calc{};
    const auto overlay = calc.FindOverlay(edgeList1, edgeList2);

    DoublyConnectedEdgeList expected;
    {
        const auto v0 = expected.InsertVertex({0, 2});
        const auto v1 = expected.InsertVertex({2, 0});
        const auto v2 = expected.InsertVertex({4, 2});
        const auto v3 = expected.InsertVertex({2, 4});

        const auto v4 = expected.InsertVertex({2, 2});
        const auto v5 = expected.InsertVertex({4, 0});
        const auto v6 = expected.InsertVertex({6, 2});
        const auto v7 = expected.InsertVertex({4, 4});

        const auto v9 = expected.InsertVertex({0, 4});
        const auto v10 = expected.InsertVertex({3, 6});

        const auto v11 = expected.InsertVertex({2, 7});
        const auto v12 = expected.InsertVertex({4, 5});
        const auto v13 = expected.InsertVertex({4, 7});
        const auto v14 = expected.InsertVertex({3, 8});

        const auto v15 = expected.InsertVertex({3, 1});
        const auto v16 = expected.InsertVertex({3, 3});

        expected.InsertEdge(v0, v1);
        expected.InsertEdge(v1, v15);
        expected.InsertEdge(v15, v2);
        expected.InsertEdge(v2, v16);
        expected.InsertEdge(v16, v3);
        expected.InsertEdge(v3, v0);

        expected.InsertEdge(v4, v15);
        expected.InsertEdge(v15, v5);
        expected.InsertEdge(v5, v6);
        expected.InsertEdge(v6, v7);
        expected.InsertEdge(v7, v16);
        expected.InsertEdge(v16, v4);

        expected.InsertEdge(v3, v9);
        expected.InsertEdge(v3, v10);
        expected.InsertEdge(v9, v10);

        expected.InsertEdge(v10, v11);
        expected.InsertEdge(v10, v12);
        expected.InsertEdge(v12, v13);
        expected.InsertEdge(v14, v13);
        expected.InsertEdge(v14, v11);
    }

    REQUIRE(AreIsomorphic(expected, overlay));
}
