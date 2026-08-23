#include <catch2/catch_test_macros.hpp>

#include "data_structures/AvlTree.hpp"

using namespace compg;

TEST_CASE("AvlTree", "[AvlTree]") {
    AvlTree<int> tree;
    const auto insert5 = tree.Insert(5);
    const auto insert4 = tree.Insert(4);
    const auto insert3 = tree.Insert(3);

    REQUIRE(insert5.second);
    REQUIRE(*insert5.first == 5);
    REQUIRE(insert4.second);
    REQUIRE(*insert4.first == 4);
    REQUIRE(insert3.second);
    REQUIRE(*insert3.first == 3);

    SECTION("Inserting element already in the tree does not create a duplicate") {
        const auto insertAgain = tree.Insert(5);
        REQUIRE_FALSE(insertAgain.second);
        REQUIRE(*insertAgain.first == 5);
    }

    SECTION("Find element in the tree") {
        const auto find = tree.Find(4);
        REQUIRE(find != tree.end());
        REQUIRE(*find == 4);
    }

    SECTION("Find element not in the tree") {
        const auto find = tree.Find(6);
        REQUIRE(find == tree.end());
    }

    SECTION("Delete element in the tree") {
        const auto deleted = tree.Erase(3);
        REQUIRE(deleted);
    }

    SECTION("Delete element not in the tree") {
        const auto deleted = tree.Erase(2);
        REQUIRE_FALSE(deleted);
    }

    SECTION("Insert does not invalid iterator") {
        const auto find5 = tree.Find(5);
        tree.Insert(1042);
        tree.Insert(1043);

        REQUIRE(*find5 == 5);

        auto it = std::next(find5);
        REQUIRE(it != tree.end());
        REQUIRE(*it == 1042);

        ++it;
        REQUIRE(it != tree.end());
        REQUIRE(*it == 1043);

        ++it;
        REQUIRE(it == tree.end());

        REQUIRE(find5 != tree.begin());
        it = std::prev(find5);
        REQUIRE(*it == 4);
        REQUIRE(it != tree.begin());

        --it;
        REQUIRE(*it == 3);
        REQUIRE(it == tree.begin());
    }

    SECTION("Delete does not invalid iterator") {
        auto it = tree.Find(3);
        tree.Erase(4);

        REQUIRE(it != tree.end());
        REQUIRE(*it == 3);
        ++it;
        REQUIRE(it != tree.end());
        REQUIRE(*it == 5);
        ++it;
        REQUIRE(it == tree.end());
    }

    SECTION("LowerBound return iterator to smallest element not less than the input") {
        tree.Insert(-10);
        auto it = tree.LowerBound(0);
        REQUIRE(it != tree.end());
        REQUIRE(*it == 3);

        it = tree.LowerBound(4);
        REQUIRE(it != tree.end());
        REQUIRE(*it == 4);
    }

    SECTION("UpperBound return iterator to smallest element greater than the input") {
        tree.Insert(10);
        auto it = tree.UpperBound(8);
        REQUIRE(it != tree.end());
        REQUIRE(*it == 10);

        it = tree.UpperBound(10);
        REQUIRE(it == tree.end());

        it = tree.UpperBound(5);
        REQUIRE(it != tree.begin());
        REQUIRE(*std::prev(it) == 5);
    }
}

TEST_CASE("AvlTree with non copy constructable type, custom comparator and projector", "[AvlTree]") {
    struct X {
        int A;
    };
    AvlTree<std::unique_ptr<X>> tree;
    auto comp = Greater{};
    auto proj = [](const auto& ptr) { return ptr->A; };
    auto insert1 = tree.Insert(std::make_unique<X>(6), comp, proj);
    auto insert2 = tree.Insert(std::make_unique<X>(4), comp, proj);
    auto insert3 = tree.Insert(std::make_unique<X>(5), comp, proj);
    auto insert4 = tree.Insert(std::make_unique<X>(4), comp, proj);

    REQUIRE(insert1.second);
    REQUIRE((*insert1.first)->A == 6);
    REQUIRE(insert2.second);
    REQUIRE((*insert2.first)->A == 4);
    REQUIRE(insert3.second);
    REQUIRE((*insert3.first)->A == 5);
    REQUIRE_FALSE(insert4.second);
    REQUIRE(insert2.first == insert4.first);

    std::vector<int> values;
    for (const auto& ptr : tree) {
        values.push_back(ptr->A);
    }
    REQUIRE(values.size() == 3);
    REQUIRE(values[0] == 6);
    REQUIRE(values[1] == 5);
    REQUIRE(values[2] == 4);

    auto delete1 = tree.Erase(5, comp, proj);
    REQUIRE(delete1);
    auto find1 = tree.Find(5, comp, proj);
    REQUIRE(find1 == tree.end());
}