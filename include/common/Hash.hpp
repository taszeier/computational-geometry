#pragma once

#include "common/Common.hpp"

#include <algorithm>
#include <cstddef>
#include <ranges>
#include <vector>

namespace compg {
    template <typename T>
    void hash_combine(std::size_t& seed, const T& value) {
        std::hash<T> hasher;
        seed ^= hasher(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
} // namespace compg

namespace std {
    template <typename T, std::size_t N>
    struct hash<std::array<T, N>> {
        size_t operator()(const std::array<T, N>& array) const noexcept {
            size_t seed{0};
            std::ranges::for_each(array, [&seed](const auto& x) { compg::hash_combine(seed, x); });

            return seed;
        }
    };

    template <typename... Args>
    struct hash<std::tuple<Args...>> {
        size_t operator()(const tuple<Args...>& tup) const noexcept {
            size_t seed{0};
            compg::for_each_in_tuple(tup, [&seed](auto, const auto& x) { compg::hash_combine(seed, x); });
            return seed;
        }
    };
} // namespace std