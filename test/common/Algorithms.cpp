#include "common/Algorithms.hpp"
#include "common/Common.hpp"
#include "common/Vertex.hpp"
#include <array>
#include <catch2/catch_test_macros.hpp>

using namespace compg;

TEST_CASE("PartitionMedian - Odd number of elements", "[PartitionMedian]") {
    std::array numbers{0, 9, 4, 1, 3};
    const auto medianIterator = PartitionMedian(numbers);

    SECTION("Returned iterator points to median") {
        REQUIRE(medianIterator != numbers.end());
        REQUIRE(*medianIterator == 3);
    }

    SECTION("Values before median iterator are not greater than median") {
        std::ranges::for_each(std::ranges::subrange(numbers.begin(), medianIterator), [medianIterator](auto v) {
            REQUIRE(v <= *medianIterator);
        });
    }

    SECTION("Values after median iterator are not less than median") {
        std::ranges::for_each(
            std::ranges::subrange(std::next(medianIterator), numbers.end()),
            [medianIterator](auto v) { REQUIRE(v >= *medianIterator); }
        );
    }
}

TEST_CASE("PartitionMedian - Even number of elements", "[PartitionMedian]") {
    std::array numbers{0, 9, 4, 1};
    const auto medianIterator = PartitionMedian(numbers);

    SECTION("Returned iterator points to the first element of the two middle values") {
        REQUIRE(medianIterator != numbers.end());
        REQUIRE(*medianIterator == 1);
    }
}

TEST_CASE("ParitionMedian - Empty input", "[PartitionMedian]") {
    std::vector<Float> empty;
    SECTION("Throws exception on empty input") {
        REQUIRE_THROWS_AS(PartitionMedian(empty), compg::Exception);
    }
}

TEST_CASE("IsSorted - Input is sorted", "[IsSorted]") {

    SECTION("Array of one element is sorted") {
        std::vector<Float> numbers{1043};
        REQUIRE(IsSorted(numbers));
    }

    SECTION("Non-strictly increasing numbers are sorted") {
        std::array numbers{0, 0, 1, 2, 4, 5, 5, 10};
        REQUIRE(IsSorted(numbers));
    }

    SECTION("Vertices are sorted using custom comparator and projector") {
        const std::array vertices{Vertex3D{10, 5, 0}, Vertex3D{-1, 3, 1}, Vertex3D{-1, 3, 2}, Vertex3D{1, 2, 3}};
        REQUIRE(IsSorted(vertices, Greater{}, CoordinateProjector<3UZ>{1UZ}));
    }

    SECTION("Empty vector is sorted") {
        const std::vector<Float> empty;
        REQUIRE(IsSorted(empty));
    }
}

TEST_CASE("IsSorted - Input is not sorted", "[IsSorted]") {
    std::array numbers{0, 1, 3, 2};
    REQUIRE_FALSE(IsSorted(numbers));
}

TEST_CASE("IsStrictlySorted - Input is strictly sorted", "[IsStrictlySorted]") {
    SECTION("Array of one element is strictly sorted") {
        std::vector<Float> numbers{1043};
        REQUIRE(IsStrictlySorted(numbers));
    }

    SECTION("Vertices are sorted using custom comparator and projector") {
        const std::array vertices{Vertex3D{10, 5, 0}, Vertex3D{-1, 3.5, 1}, Vertex3D{-1, 3, 2}, Vertex3D{1, 2, 3}};
        REQUIRE(IsStrictlySorted(vertices, Greater{}, CoordinateProjector<3UZ>{1UZ}));
    }

    SECTION("Empty vector is strictly sorted") {
        const std::vector<Float> empty;
        REQUIRE(IsSorted(empty));
    }
}

TEST_CASE("IsStrictlySorted - Input is not strictly sorted", "[IsStrictlySorted]") {
    SECTION("Array of non-sorted numbers is not strictly sorted") {
        const std::array numbers{0, 3, 2};
        REQUIRE_FALSE(IsStrictlySorted(numbers));
    }

    SECTION("Sorted numbers with duplicates is not strictly sorted") {
        const std::array numbers{0, 0, 1, 2, 4, 5, 5, 10};
        REQUIRE_FALSE(IsStrictlySorted(numbers));
    }
}

TEST_CASE("BisectLeftMany", "[BisectLeftMany]") {
    const std::vector queries{0, 3, 5, 9};

    SECTION("All queries have a valid bisect index") {
        const auto bisectResult = BisectLeftMany(std::vector{-1, 4, 10}, queries);
        REQUIRE(bisectResult.size() == queries.size());
        REQUIRE(bisectResult[0] == 1);
        REQUIRE(bisectResult[1] == 1);
        REQUIRE(bisectResult[2] == 2);
        REQUIRE(bisectResult[3] == 2);
    }

    SECTION("No query has a valid bisect index") {
        const auto bisectResult = BisectLeftMany(std::vector{-3, -2}, queries);
        REQUIRE(bisectResult.size() == queries.size());
        REQUIRE(bisectResult[0] == 2);
        REQUIRE(bisectResult[1] == 2);
        REQUIRE(bisectResult[2] == 2);
        REQUIRE(bisectResult[3] == 2);
    }

    SECTION("All queries are not greater than the smallest value") {
        const auto bisectResult = BisectLeftMany(std::vector{9, 1042, 1043}, queries);
        REQUIRE(bisectResult.size() == queries.size());
        REQUIRE(bisectResult[0] == 0);
        REQUIRE(bisectResult[1] == 0);
        REQUIRE(bisectResult[2] == 0);
        REQUIRE(bisectResult[3] == 0);
    }

    SECTION("Throws exception when the queries are not sorted") {
        REQUIRE_THROWS_AS(BisectLeftMany(std::vector{2}, std::vector{2, 1}), compg::Exception);
    }

    SECTION("Throws exception when the values are not sorted") {
        REQUIRE_THROWS_AS(BisectLeftMany(std::vector{2, 1}, queries), compg::Exception);
    }

    SECTION("Uses custom comparator and projector") {
        std::vector<Vertex2D> queriesV2D{{3, -1}, {2, 1}};
        std::vector<Vertex2D> values{{2.71, -1}, {2.5, 10}};
        const auto bisectResult = BisectLeftMany(values, queriesV2D, Greater{}, CoordinateProjector<2>{0});
        REQUIRE(bisectResult.size() == 2UZ);
        REQUIRE(bisectResult[0] == 0);
        REQUIRE(bisectResult[1] == 2);
    }
}

TEST_CASE("ArgSort", "[ArgSort]") {
    const std::array numbers{4, 2, 10, 13, 11, 1043};
    const auto argSort = ArgSort(numbers);

    REQUIRE(argSort.size() == 6);
    REQUIRE(argSort[0] == 1);
    REQUIRE(argSort[1] == 0);
    REQUIRE(argSort[2] == 2);
    REQUIRE(argSort[3] == 4);
    REQUIRE(argSort[4] == 3);
    REQUIRE(argSort[5] == 5);
}