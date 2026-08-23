#include "data_structures/DisjointSetUnion.hpp"
#include "common/Error.hpp"

#include <ranges>

namespace compg {
    std::size_t DisjointSetUnion::Find(std::size_t item) {
        COMPG_ASSERT(item < Parent.size(), CreateOutOfRangeMessage("item", item, Size()));

        while (Parent.at(item) != item) {
            const auto grandParent = Parent.at(Parent.at(item));
            Parent.at(item) = grandParent;
            item = grandParent;
        }
        return item;
    }

    bool DisjointSetUnion::Union(std::size_t item1, std::size_t item2) {
        COMPG_ASSERT(item1 < Size(), CreateOutOfRangeMessage("item1", item1, Size()));
        COMPG_ASSERT(item2 < Size(), CreateOutOfRangeMessage("item2", item2, Size()));

        auto root1 = Find(item1);
        auto root2 = Find(item2);

        if (root1 == root2) {
            return false;
        }

        if (Rank.at(root1) < Rank.at(root2)) {
            std::swap(root1, root2);
        }
        Parent.at(root2) = root1;
        if (Rank.at(root1) == Rank.at(root2)) {
            ++Rank.at(root1);
        }

        return true;
    }

    auto DisjointSetUnion::FindDisjointSets() -> sets_type {
        sets_type sets{};
        std::ranges::for_each(std::views::iota(0UZ, Size()), [&sets, this](auto i) {
            const auto root = Find(i);
            sets[root].insert(i);
        });

        return sets;
    }
} // namespace compg