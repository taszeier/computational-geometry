#pragma once

#include <cstddef>
#include <eigen3/Eigen/Core>
#include <iterator>
#include <random>
#include <ranges>

namespace compg {

    inline constexpr std::size_t DEFAULT_SEED = 0UZ;

    class RandomState {
    public:
        explicit RandomState(std::size_t seed = DEFAULT_SEED)
            : Seed{seed}
            , Generator{Seed} {}

        template <typename IntegerType>
        IntegerType RandomInt(IntegerType lower, IntegerType upper) {
            std::uniform_int_distribution<IntegerType> distribution{lower, upper};
            return distribution(Generator);
        }

        template <typename IntegerType>
        IntegerType RandomInt(IntegerType upper) {
            return RandomInt(static_cast<IntegerType>(0), upper);
        }

        template <typename RealType>
        RealType Random(RealType lower, RealType upper) {
            std::uniform_real_distribution<RealType> distribution{lower, upper};
            return distribution(Generator);
        }

    private:
        std::size_t Seed;
        std::mt19937 Generator;
    };

    template <typename Integer>
    Integer RandomInt(Integer lower, Integer upper, std::size_t seed = DEFAULT_SEED) {
        return RandomState{seed}.RandomInt(lower, upper);
    }

    template <typename Iterator>
    void RandomPermutation(Iterator begin, Iterator end, std::size_t seed = DEFAULT_SEED) {
        if (begin == end) {
            return;
        }
        RandomState randomState{seed};
        for (auto insertIndex : std::views::iota(1, std::distance(begin, end)) | std::views::reverse) {
            auto swapIndex = randomState.RandomInt(insertIndex);
            std::swap(*std::next(begin, insertIndex), *std::next(begin, swapIndex));
        }
    }

    void RandomPermutation(std::ranges::range auto& range, std::size_t seed = DEFAULT_SEED) {
        RandomPermutation(range.begin(), range.end(), seed);
    }

    inline std::vector<std::size_t> RandomIndexPermutation(std::size_t size, std::size_t seed = DEFAULT_SEED) {
        std::vector<std::size_t> indices(size);
        std::ranges::iota(indices, 0UZ);
        RandomPermutation(indices, seed);
        return indices;
    }

    std::vector<std::size_t>
    RandomIndexPermutation(const std::ranges::range auto& range, std::size_t seed = DEFAULT_SEED) {
        return RandomIndexPermutation(range.size(), seed);
    }
} // namespace compg
