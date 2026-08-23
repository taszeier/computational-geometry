#include <catch2/catch_test_macros.hpp>

#include "math/Math.hpp"
#include "window_query/PrioritySearchTree.hpp"

using namespace compg;

TEST_CASE("PrioritySearchTree", "[PrioritySearchTree]") {
    const std::vector<Vertex2D> vertices{{1, 1}, {3, 3}, {0, 2}};

    PrioritySearchTree tree{vertices};
    const auto queryResult = tree.Query({Interval<Float>{1.5, 4.0}, 1.5});
    REQUIRE(queryResult.size() == 1);
    REQUIRE(AreEqual(queryResult.at(0).Value, {0, 2}));
    REQUIRE(queryResult.at(0).Index == 2);
}