#pragma once

#include "binary_space_partition/BspTree.hpp"
#include "common/Random.hpp"

namespace compg {
    class BspCalculator2D {
    public:
        explicit BspCalculator2D(Float epsilon = EPSILON, std::size_t seed = DEFAULT_SEED)
            : Epsilon{epsilon}
            , Seed{seed} {}

        [[nodiscard]] BspTree2D FindPartition(const std::vector<LineSegment2D>& segments) const;
        [[nodiscard]] BspTree2D FindLowDensityPartition(const std::vector<LineSegment2D>& segments) const;

    private:
        void SetLeaves(const std::vector<LineSegment2D>& segments, std::unique_ptr<BspTreeNode2D>& root) const;

    private:
        Float Epsilon;
        std::size_t Seed;
    };
} // namespace compg