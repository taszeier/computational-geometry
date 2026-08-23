#include <catch2/catch_test_macros.hpp>

#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include "math/Conversions.hpp"
#include "quad_tree/CircuitQuadTree.hpp"

using namespace compg;

TEST_CASE("Create mesh from quad tree", "[CircuitQuadTree]") {
    const CircuitSegment segment{{{1, 31}, {2, 30}}};
    CircuitQuadTree quadTree{{segment}, 5UZ};

    const auto mesh = quadTree.CreateMesh();
    DoublyConnectedEdgeList expected;
    {
        const auto v0 = expected.InsertVertex({0, 0});
        const auto v1 = expected.InsertVertex({8, 0});
        const auto v2 = expected.InsertVertex({16, 0});
        const auto v3 = expected.InsertVertex({32, 0});
        const auto v4 = expected.InsertVertex({32, 16});
        const auto v5 = expected.InsertVertex({32, 24});
        const auto v6 = expected.InsertVertex({32, 32});
        const auto v7 = expected.InsertVertex({24, 32});
        const auto v8 = expected.InsertVertex({16, 32});
        const auto v9 = expected.InsertVertex({12, 32});
        const auto v10 = expected.InsertVertex({8, 32});
        const auto v11 = expected.InsertVertex({6, 32});
        const auto v12 = expected.InsertVertex({4, 32});
        const auto v13 = expected.InsertVertex({3, 32});
        const auto v14 = expected.InsertVertex({2, 32});
        const auto v15 = expected.InsertVertex({1, 32});
        const auto v16 = expected.InsertVertex({0, 32});
        const auto v17 = expected.InsertVertex({0, 31});
        const auto v18 = expected.InsertVertex({0, 30});
        const auto v19 = expected.InsertVertex({0, 29});
        const auto v20 = expected.InsertVertex({0, 28});
        const auto v21 = expected.InsertVertex({0, 26});
        const auto v22 = expected.InsertVertex({0, 24});
        const auto v23 = expected.InsertVertex({0, 20});
        const auto v24 = expected.InsertVertex({0, 16});
        const auto v25 = expected.InsertVertex({0, 8});

        expected.InsertEdge(v0, v1);
        expected.InsertEdge(v1, v2);
        expected.InsertEdge(v2, v3);
        expected.InsertEdge(v3, v4);
        expected.InsertEdge(v4, v5);
        expected.InsertEdge(v5, v6);
        expected.InsertEdge(v6, v7);
        expected.InsertEdge(v7, v8);
        expected.InsertEdge(v8, v9);
        expected.InsertEdge(v9, v10);
        expected.InsertEdge(v10, v11);
        expected.InsertEdge(v11, v12);
        expected.InsertEdge(v12, v13);
        expected.InsertEdge(v13, v14);
        expected.InsertEdge(v14, v15);
        expected.InsertEdge(v15, v16);
        expected.InsertEdge(v16, v17);
        expected.InsertEdge(v17, v18);
        expected.InsertEdge(v18, v19);
        expected.InsertEdge(v19, v20);
        expected.InsertEdge(v20, v21);
        expected.InsertEdge(v21, v22);
        expected.InsertEdge(v22, v23);
        expected.InsertEdge(v23, v24);
        expected.InsertEdge(v24, v25);
        expected.InsertEdge(v25, v0);

        const auto v26 = expected.InsertVertex({16, 16});
        const auto v27 = expected.InsertVertex({16, 8});
        const auto v28 = expected.InsertVertex({24, 16});
        const auto v29 = expected.InsertVertex({16, 24});
        const auto v30 = expected.InsertVertex({16, 28});
        const auto v31 = expected.InsertVertex({8, 16});
        const auto v32 = expected.InsertVertex({4, 16});

        expected.InsertEdge(v26, v27);
        expected.InsertEdge(v27, v2);
        expected.InsertEdge(v26, v28);
        expected.InsertEdge(v28, v4);
        expected.InsertEdge(v26, v29);
        expected.InsertEdge(v29, v30);
        expected.InsertEdge(v30, v8);
        expected.InsertEdge(v26, v31);
        expected.InsertEdge(v31, v32);
        expected.InsertEdge(v32, v24);

        const auto v33 = expected.InsertVertex({24, 24});
        expected.InsertEdge(v33, v28);
        expected.InsertEdge(v33, v5);
        expected.InsertEdge(v33, v7);
        expected.InsertEdge(v33, v29);

        const auto v34 = expected.InsertVertex({8, 8});
        expected.InsertEdge(v34, v1);
        expected.InsertEdge(v34, v27);
        expected.InsertEdge(v34, v31);
        expected.InsertEdge(v34, v25);

        const auto v35 = expected.InsertVertex({8, 24});
        const auto v36 = expected.InsertVertex({8, 20});
        const auto v37 = expected.InsertVertex({12, 24});
        const auto v38 = expected.InsertVertex({8, 28});
        const auto v39 = expected.InsertVertex({8, 30});
        const auto v40 = expected.InsertVertex({4, 24});
        const auto v41 = expected.InsertVertex({2, 24});

        expected.InsertEdge(v35, v36);
        expected.InsertEdge(v36, v31);
        expected.InsertEdge(v35, v37);
        expected.InsertEdge(v37, v29);
        expected.InsertEdge(v35, v38);
        expected.InsertEdge(v38, v39);
        expected.InsertEdge(v39, v10);
        expected.InsertEdge(v35, v40);
        expected.InsertEdge(v40, v41);
        expected.InsertEdge(v41, v22);

        const auto v42 = expected.InsertVertex({12, 28});
        expected.InsertEdge(v42, v37);
        expected.InsertEdge(v42, v30);
        expected.InsertEdge(v42, v9);
        expected.InsertEdge(v42, v38);

        const auto v43 = expected.InsertVertex({4, 20});
        expected.InsertEdge(v43, v32);
        expected.InsertEdge(v43, v36);
        expected.InsertEdge(v43, v40);
        expected.InsertEdge(v43, v23);

        const auto v44 = expected.InsertVertex({4, 28});
        const auto v45 = expected.InsertVertex({4, 26});
        const auto v46 = expected.InsertVertex({6, 28});
        const auto v47 = expected.InsertVertex({4, 29});
        const auto v48 = expected.InsertVertex({4, 30});
        const auto v49 = expected.InsertVertex({4, 31});
        const auto v50 = expected.InsertVertex({3, 28});
        const auto v51 = expected.InsertVertex({2, 28});
        const auto v52 = expected.InsertVertex({1, 28});

        expected.InsertEdge(v44, v45);
        expected.InsertEdge(v45, v40);
        expected.InsertEdge(v44, v46);
        expected.InsertEdge(v46, v38);
        expected.InsertEdge(v44, v47);
        expected.InsertEdge(v47, v48);
        expected.InsertEdge(v48, v49);
        expected.InsertEdge(v49, v12);
        expected.InsertEdge(v44, v50);
        expected.InsertEdge(v50, v51);
        expected.InsertEdge(v51, v52);
        expected.InsertEdge(v52, v20);

        const auto v53 = expected.InsertVertex({6, 30});
        expected.InsertEdge(v53, v46);
        expected.InsertEdge(v53, v39);
        expected.InsertEdge(v53, v11);
        expected.InsertEdge(v53, v48);

        const auto v54 = expected.InsertVertex({2, 26});
        expected.InsertEdge(v54, v41);
        expected.InsertEdge(v54, v45);
        expected.InsertEdge(v54, v51);
        expected.InsertEdge(v54, v21);

        const auto v55 = expected.InsertVertex({2, 30});
        const auto v56 = expected.InsertVertex({2, 29});
        const auto v57 = expected.InsertVertex({3, 30});
        const auto v58 = expected.InsertVertex({2, 31});
        const auto v59 = expected.InsertVertex({1, 30});

        expected.InsertEdge(v55, v56);
        expected.InsertEdge(v56, v51);
        expected.InsertEdge(v55, v57);
        expected.InsertEdge(v57, v48);
        expected.InsertEdge(v55, v58);
        expected.InsertEdge(v58, v14);
        expected.InsertEdge(v55, v59);
        expected.InsertEdge(v59, v18);

        const auto v60 = expected.InsertVertex({3, 29});
        expected.InsertEdge(v60, v50);
        expected.InsertEdge(v60, v47);
        expected.InsertEdge(v60, v57);
        expected.InsertEdge(v60, v56);

        const auto v61 = expected.InsertVertex({3, 31});
        expected.InsertEdge(v61, v57);
        expected.InsertEdge(v61, v49);
        expected.InsertEdge(v61, v13);
        expected.InsertEdge(v61, v58);

        const auto v62 = expected.InsertVertex({1, 29});
        expected.InsertEdge(v62, v52);
        expected.InsertEdge(v62, v56);
        expected.InsertEdge(v62, v59);
        expected.InsertEdge(v62, v19);

        const auto v63 = expected.InsertVertex({1, 31});
        expected.InsertEdge(v63, v59);
        expected.InsertEdge(v63, v58);
        expected.InsertEdge(v63, v15);
        expected.InsertEdge(v63, v17);

        expected.InsertEdge(v17, v15);
        expected.InsertEdge(v18, v63);
        expected.InsertEdge(v63, v14);
        expected.InsertEdge(v19, v59);
        expected.InsertEdge(v55, v63);
        expected.InsertEdge(v58, v13);
        expected.InsertEdge(v20, v62);
        expected.InsertEdge(v62, v55);
        expected.InsertEdge(v55, v61);
        expected.InsertEdge(v61, v12);

        expected.InsertEdge(v52, v56);
        expected.InsertEdge(v56, v57);
        expected.InsertEdge(v57, v49);
        expected.InsertEdge(v51, v60);
        expected.InsertEdge(v60, v48);
        expected.InsertEdge(v50, v47);

        {
            const auto v = expected.InsertVertex({3, 27});
            expected.InsertEdge(v, v54);
            expected.InsertEdge(v, v45);
            expected.InsertEdge(v, v44);
            expected.InsertEdge(v, v50);
            expected.InsertEdge(v, v51);
        }

        {
            const auto v = expected.InsertVertex({1, 27});
            expected.InsertEdge(v, v21);
            expected.InsertEdge(v, v54);
            expected.InsertEdge(v, v51);
            expected.InsertEdge(v, v52);
            expected.InsertEdge(v, v20);
        }
        expected.InsertEdge(v22, v54);
        expected.InsertEdge(v41, v45);

        expected.InsertEdge(v53, v10);
        {
            const auto v = expected.InsertVertex({5, 31});
            expected.InsertEdge(v, v48);
            expected.InsertEdge(v, v53);
            expected.InsertEdge(v, v11);
            expected.InsertEdge(v, v12);
            expected.InsertEdge(v, v49);
        }
        expected.InsertEdge(v53, v10);
        {
            const auto v = expected.InsertVertex({5, 29});
            expected.InsertEdge(v, v44);
            expected.InsertEdge(v, v46);
            expected.InsertEdge(v, v53);
            expected.InsertEdge(v, v48);
            expected.InsertEdge(v, v47);
        }
        expected.InsertEdge(v46, v39);

        {
            const auto v = expected.InsertVertex({6, 26});
            expected.InsertEdge(v, v40);
            expected.InsertEdge(v, v35);
            expected.InsertEdge(v, v38);
            expected.InsertEdge(v, v46);
            expected.InsertEdge(v, v44);
            expected.InsertEdge(v, v45);
        }

        expected.InsertEdge(v43, v35);
        {
            const auto v = expected.InsertVertex({2, 22});
            expected.InsertEdge(v, v23);
            expected.InsertEdge(v, v43);
            expected.InsertEdge(v, v40);
            expected.InsertEdge(v, v41);
            expected.InsertEdge(v, v22);
        }
        expected.InsertEdge(v24, v43);
        expected.InsertEdge(v32, v36);

        expected.InsertEdge(v42, v8);
        {
            const auto v = expected.InsertVertex({10, 30});
            expected.InsertEdge(v, v38);
            expected.InsertEdge(v, v42);
            expected.InsertEdge(v, v9);
            expected.InsertEdge(v, v10);
            expected.InsertEdge(v, v39);
        }
        expected.InsertEdge(v35, v42);
        expected.InsertEdge(v37, v30);

        {
            const auto v = expected.InsertVertex({12, 20});
            expected.InsertEdge(v, v31);
            expected.InsertEdge(v, v26);
            expected.InsertEdge(v, v29);
            expected.InsertEdge(v, v37);
            expected.InsertEdge(v, v35);
            expected.InsertEdge(v, v36);
        }

        expected.InsertEdge(v34, v26);
        {
            const auto v = expected.InsertVertex({4, 12});
            expected.InsertEdge(v, v25);
            expected.InsertEdge(v, v34);
            expected.InsertEdge(v, v31);
            expected.InsertEdge(v, v32);
            expected.InsertEdge(v, v24);
        }
        expected.InsertEdge(v0, v34);
        expected.InsertEdge(v1, v27);

        expected.InsertEdge(v33, v6);
        {
            const auto v = expected.InsertVertex({20, 28});
            expected.InsertEdge(v, v29);
            expected.InsertEdge(v, v33);
            expected.InsertEdge(v, v7);
            expected.InsertEdge(v, v8);
            expected.InsertEdge(v, v30);
        }
        expected.InsertEdge(v26, v33);
        expected.InsertEdge(v28, v5);

        {
            const auto v = expected.InsertVertex({24, 8});
            expected.InsertEdge(v, v2);
            expected.InsertEdge(v, v3);
            expected.InsertEdge(v, v4);
            expected.InsertEdge(v, v28);
            expected.InsertEdge(v, v26);
            expected.InsertEdge(v, v27);
        }
    }

    REQUIRE(AreIsomorphic(mesh, expected));
}