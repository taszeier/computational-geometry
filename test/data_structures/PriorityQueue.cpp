#include <catch2/catch_test_macros.hpp>

#include "data_structures/PriorityQueue.hpp"

using namespace compg;

TEST_CASE("PriorityQueue", "[PriorityQueue]") {
    PriorityQueue<int> queue;
    queue.Insert(4);
    queue.Insert(0);
    queue.Insert(10);
    queue.Insert(2);
    queue.Insert(6);

    SECTION("The correct element is at the top after popping") {
        REQUIRE(queue.Top() == 10);
        queue.Pop();
        REQUIRE(queue.Top() == 6);
        queue.Pop();
        REQUIRE(queue.Top() == 4);
        queue.Pop();
        REQUIRE(queue.Top() == 2);
        queue.Pop();
        REQUIRE(queue.Top() == 0);
        queue.Pop();
        REQUIRE(queue.IsEmpty());
    }

    SECTION("An element is correctly erased from the queue") {
        queue.Erase(6);
        queue.Erase(4);
        REQUIRE(queue.Top() == 10);
        queue.Pop();
        REQUIRE(queue.Top() == 2);
        queue.Pop();
        REQUIRE(queue.Top() == 0);
        queue.Pop();
        REQUIRE(queue.IsEmpty());
    }

    SECTION("An element is inserted multiple times") {
        queue.Insert(6);
        REQUIRE(queue.Size() == 6);
        REQUIRE(queue.Top() == 10);
        queue.Pop();
        REQUIRE(queue.Top() == 6);
        queue.Pop();
        REQUIRE(queue.Top() == 6);
        queue.Pop();
        REQUIRE(queue.Top() == 4);
    }

    SECTION("Elements are inserted correctly after erasing an element") {
        queue.Erase(6);
        queue.Insert(11);
        queue.Insert(6);
        queue.Insert(7);

        REQUIRE(queue.Top() == 11);
        queue.Pop();
        REQUIRE(queue.Top() == 10);
        queue.Pop();
        REQUIRE(queue.Top() == 7);
        queue.Pop();
        REQUIRE(queue.Top() == 6);
        queue.Pop();
        REQUIRE(queue.Top() == 4);
        queue.Pop();
        REQUIRE(queue.Top() == 2);
        queue.Pop();
        REQUIRE(queue.Top() == 0);
        queue.Pop();
        REQUIRE(queue.IsEmpty());
    }
}

TEST_CASE("Priority queue is initialized correctly", "[PriorityQueue]") {
    constexpr std::array numbers{0, 4, 2, 0, 1, 9, 5, 0, 2};
    PriorityQueue<int> queue{numbers};

    REQUIRE(queue.Size() == 9);
    REQUIRE(queue.Top() == 9);
    queue.Pop();
    REQUIRE(queue.Top() == 5);
    queue.Pop();
    REQUIRE(queue.Top() == 4);
    queue.Pop();
    REQUIRE(queue.Top() == 2);
    queue.Pop();
    REQUIRE(queue.Top() == 2);
    queue.Pop();
    REQUIRE(queue.Top() == 1);
    queue.Pop();
    REQUIRE(queue.Top() == 0);
    queue.Pop();
    REQUIRE(queue.Top() == 0);
    queue.Pop();
    REQUIRE(queue.Top() == 0);
    queue.Pop();
    REQUIRE(queue.IsEmpty());
}