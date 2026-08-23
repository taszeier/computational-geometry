#pragma once

#include "binary_space_partition/BspTree.hpp"
#include "common/Random.hpp"

namespace compg {
    class BspCalculator3D {
    public:
        explicit BspCalculator3D(Float epsilon = EPSILON, std::size_t seed = DEFAULT_SEED)
            : Epsilon{epsilon}
            , Seed{seed} {}

        [[nodiscard]] BspTree3D FindPartition(const std::vector<Triangle3D>& triangles) const;
        [[nodiscard]] BspTree3D FindLowDensityPartition(const std::vector<Triangle3D>& triangles) const;

    private:
        void SetLeaves(const std::vector<Triangle3D>& triangles, std::unique_ptr<BspTreeNode3D>& root) const;

    private:
        Float Epsilon;
        std::size_t Seed;
    };
} // namespace compg