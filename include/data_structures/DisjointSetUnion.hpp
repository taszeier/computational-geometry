#pragma once

#include "common/Common.hpp"
#include <unordered_map>
#include <unordered_set>

namespace compg {
    class DisjointSetUnion {
    public:
        using sets_type = std::unordered_map<std::size_t, std::unordered_set<std::size_t>>;

        /**
         * @brief Create a group of disjoint sets.
         * @param size The number of items.
         */
        explicit DisjointSetUnion(std::size_t size)
            : Parent(size)
            , Rank(size, 0) {
            std::ranges::iota(Parent, 0UZ);
        }

        /**
         * @brief Find the representative of an item.
         * @param item The item.
         * @return The representative.
         */
        std::size_t Find(std::size_t item);

        /**
         * @brief Combine two sets given an element from each.
         * @param item1 An item from set 1.
         * @param item2 An item from set 2.
         * @return Whether the items were in different sets.
         */
        bool Union(std::size_t item1, std::size_t item2);

        /**
         * @brief Find the disjoint sets and their representatives.
         * @return The disjoint sets.
         */
        sets_type FindDisjointSets();

        [[nodiscard]] std::size_t Size() const {
            return Parent.size();
        }

    private:
        std::vector<std::size_t> Parent;
        std::vector<std::size_t> Rank;
    };
} // namespace compg