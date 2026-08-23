#include <catch2/catch_test_macros.hpp>

#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include <set>

using namespace compg;

TEST_CASE(
    "FindIsomorphism - Two edge lists are isomorphic even when vertex and edge indices are different",
    "[FindIsomorphism]"
) {
    DoublyConnectedEdgeList from;
    const auto fv0 = from.InsertVertex({0, 0});
    const auto fv1 = from.InsertVertex({1, 0});
    const auto fv2 = from.InsertVertex({1, 1});
    const auto fv3 = from.InsertVertex({0, 1});

    const auto fe0 = from.InsertEdge(fv0, fv1);
    const auto fe1 = from.InsertEdge(fv1, fv2);
    const auto fe2 = from.InsertEdge(fv2, fv3);
    const auto fe3 = from.InsertEdge(fv3, fv0);

    DoublyConnectedEdgeList to;
    const auto tv2 = to.InsertVertex({1, 1});
    const auto tv0 = to.InsertVertex({0, 0});
    const auto tv3 = to.InsertVertex({0, 1});
    const auto tv1 = to.InsertVertex({1, 0});

    const auto te3 = to.InsertEdge(tv3, tv0);
    const auto te0 = to.InsertEdge(tv0, tv1);
    const auto te1 = to.InsertEdge(tv1, tv2);
    const auto te2 = to.InsertEdge(tv2, tv3);

    const auto maybeIsomorphism = FindIsomorphism(from, to);
    REQUIRE(maybeIsomorphism.has_value());
    const auto& [vertexIsomorphism, edgeIsomorphism] = maybeIsomorphism.value();

    REQUIRE(vertexIsomorphism.size() == 4);
    REQUIRE(tv0 == vertexIsomorphism.at(fv0));
    REQUIRE(tv1 == vertexIsomorphism.at(fv1));
    REQUIRE(tv2 == vertexIsomorphism.at(fv2));
    REQUIRE(tv3 == vertexIsomorphism.at(fv3));

    REQUIRE(edgeIsomorphism.size() == 8);
    REQUIRE(te0 == edgeIsomorphism.at(fe0));
    REQUIRE(te1 == edgeIsomorphism.at(fe1));
    REQUIRE(te2 == edgeIsomorphism.at(fe2));
    REQUIRE(te3 == edgeIsomorphism.at(fe3));
    REQUIRE(to.GetTwinIndex(te0) == edgeIsomorphism.at(from.GetTwinIndex(fe0)));
    REQUIRE(to.GetTwinIndex(te1) == edgeIsomorphism.at(from.GetTwinIndex(fe1)));
    REQUIRE(to.GetTwinIndex(te2) == edgeIsomorphism.at(from.GetTwinIndex(fe2)));
    REQUIRE(to.GetTwinIndex(te3) == edgeIsomorphism.at(from.GetTwinIndex(fe3)));
}

TEST_CASE("AreIsomorphic - returns false when the edge lists are not isometric", "AreIsometric") {
    DoublyConnectedEdgeList edgeList1;
    const auto e1v0 = edgeList1.InsertVertex({0, 0});
    const auto e1v1 = edgeList1.InsertVertex({1, 0});
    const auto e1v2 = edgeList1.InsertVertex({1, 1});
    const auto e1v3 = edgeList1.InsertVertex({0, 1});

    edgeList1.InsertEdge(e1v0, e1v1);
    edgeList1.InsertEdge(e1v1, e1v2);
    edgeList1.InsertEdge(e1v2, e1v3);
    edgeList1.InsertEdge(e1v3, e1v0);
    edgeList1.InsertEdge(e1v0, e1v2);

    DoublyConnectedEdgeList edgeList2;
    const auto e2v0 = edgeList2.InsertVertex({0, 0});
    const auto e2v1 = edgeList2.InsertVertex({1, 0});
    const auto e2v2 = edgeList2.InsertVertex({1, 1});
    const auto e2v3 = edgeList2.InsertVertex({0, 1});

    edgeList2.InsertEdge(e2v0, e2v1);
    edgeList2.InsertEdge(e2v1, e2v2);
    edgeList2.InsertEdge(e2v2, e2v3);
    edgeList2.InsertEdge(e2v3, e2v0);
    edgeList2.InsertEdge(e2v1, e2v3);

    REQUIRE_FALSE(AreIsomorphic(edgeList1, edgeList2));
}

TEST_CASE(
    "UpdateFaces"
    "[UpdateFaces]"
) {
    DoublyConnectedEdgeList edgeList;
    const auto v0 = edgeList.InsertVertex({0, 0});
    const auto v1 = edgeList.InsertVertex({2, 0});
    const auto v2 = edgeList.InsertVertex({2, 2});
    const auto v3 = edgeList.InsertVertex({1, 4});
    const auto v4 = edgeList.InsertVertex({0, 2});

    const auto v5 = edgeList.InsertVertex({4, 1});
    const auto v6 = edgeList.InsertVertex({6, 2});
    const auto v7 = edgeList.InsertVertex({5, 3});

    const auto v8 = edgeList.InsertVertex({-10, -10});
    const auto v9 = edgeList.InsertVertex({10, -10});
    const auto v10 = edgeList.InsertVertex({10, 10});
    const auto v11 = edgeList.InsertVertex({-10, 10});

    const auto e0 = edgeList.InsertEdge(v0, v1);
    const auto e1 = edgeList.InsertEdge(v1, v2);
    const auto e2 = edgeList.InsertEdge(v2, v3);
    const auto e3 = edgeList.InsertEdge(v3, v4);
    const auto e4 = edgeList.InsertEdge(v4, v0);
    const auto e5 = edgeList.InsertEdge(v2, v4);

    const auto e6 = edgeList.InsertEdge(v5, v6);
    const auto e7 = edgeList.InsertEdge(v6, v7);
    const auto e8 = edgeList.InsertEdge(v7, v5);

    const auto e9 = edgeList.InsertEdge(v8, v9);
    const auto e10 = edgeList.InsertEdge(v9, v10);
    const auto e11 = edgeList.InsertEdge(v10, v11);
    const auto e12 = edgeList.InsertEdge(v11, v8);

    UpdateFaces(edgeList);

    const std::set face0Edges{e0, e1, e4, e5};
    const std::set face1Edges{e2, e3, edgeList.GetTwinIndex(e5)};
    const std::set face2Edges{e6, e7, e8};
    const std::set face3OuterEdges{e9, e10, e11, e12};
    const std::set face3InnerEdges{edgeList.GetTwinIndex(e0), edgeList.GetTwinIndex(e1), edgeList.GetTwinIndex(e2),
                                   edgeList.GetTwinIndex(e3), edgeList.GetTwinIndex(e4), edgeList.GetTwinIndex(e6),
                                   edgeList.GetTwinIndex(e7), edgeList.GetTwinIndex(e8)};
    const std::set face4Edges{
        edgeList.GetTwinIndex(e9), edgeList.GetTwinIndex(e10), edgeList.GetTwinIndex(e11), edgeList.GetTwinIndex(e12)
    };

    const auto face0 = edgeList.GetFace(edgeList.GetFaceIndex(e0));
    REQUIRE(face0.OuterComponent.has_value());
    REQUIRE(face0Edges.contains(face0.OuterComponent.value()));
    REQUIRE(face0.InnerComponents.empty());

    const auto face1 = edgeList.GetFace(edgeList.GetFaceIndex(e2));
    REQUIRE(face1.OuterComponent.has_value());
    REQUIRE(face1Edges.contains(face1.OuterComponent.value()));
    REQUIRE(face1.InnerComponents.empty());

    const auto face2 = edgeList.GetFace(edgeList.GetFaceIndex(e6));
    REQUIRE(face2.OuterComponent.has_value());
    REQUIRE(face2Edges.contains(face2.OuterComponent.value()));
    REQUIRE(face2.InnerComponents.empty());

    const auto face3 = edgeList.GetFace(edgeList.GetFaceIndex(e9));
    REQUIRE(face3.OuterComponent.has_value());
    REQUIRE(face3OuterEdges.contains(face3.OuterComponent.value()));
    REQUIRE(face3.InnerComponents.size() == 2);
    REQUIRE(face3InnerEdges.contains(face3.InnerComponents[0]));
    REQUIRE(face3InnerEdges.contains(face3.InnerComponents[1]));

    const auto face4 = edgeList.GetFace(edgeList.GetFaceIndex(edgeList.GetTwinIndex(e9)));
    REQUIRE_FALSE(face4.OuterComponent.has_value());
    REQUIRE(face4.InnerComponents.size() == 1);
    REQUIRE(face4Edges.contains(face4.InnerComponents[0]));

    const auto face0EdgesFaces = face0Edges
                                 | std::views::transform([&edgeList](auto e) { return edgeList.GetFaceIndex(e); })
                                 | std::ranges::to<std::set>();
    REQUIRE(face0EdgesFaces.size() == 1);
    const auto face1EdgesFaces = face1Edges
                                 | std::views::transform([&edgeList](auto e) { return edgeList.GetFaceIndex(e); })
                                 | std::ranges::to<std::set>();
    REQUIRE(face1EdgesFaces.size() == 1);
    const auto face2EdgesFaces = face2Edges
                                 | std::views::transform([&edgeList](auto e) { return edgeList.GetFaceIndex(e); })
                                 | std::ranges::to<std::set>();
    REQUIRE(face2EdgesFaces.size() == 1);
    auto face3Edges{face3InnerEdges};
    std::ranges::copy(face3OuterEdges, std::inserter(face3Edges, face3Edges.end()));
    const auto face3EdgesFaces = face3Edges
                                 | std::views::transform([&edgeList](auto e) { return edgeList.GetFaceIndex(e); })
                                 | std::ranges::to<std::set>();
    REQUIRE(face3EdgesFaces.size() == 1);
    const auto face4EdgesFaces = face4Edges
                                 | std::views::transform([&edgeList](auto e) { return edgeList.GetFaceIndex(e); })
                                 | std::ranges::to<std::set>();
    REQUIRE(face4EdgesFaces.size() == 1);

    const std::set facesIndices{
        *face0EdgesFaces.begin(), *face1EdgesFaces.begin(), *face2EdgesFaces.begin(),
        *face3EdgesFaces.begin(), *face4EdgesFaces.begin(),
    };

    REQUIRE(facesIndices.size() == 5);
    REQUIRE(edgeList.NumFaces() == 5);
}