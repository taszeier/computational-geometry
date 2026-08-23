#pragma once

#include "common/Random.hpp"
#include "math/primitives/Ball.hpp"

namespace compg {
    class MinimumDiscCalculator {
    public:
        [[nodiscard]] Ball<2UZ>
        FindMinimumDisc(const std::vector<Vertex2D>& vertices, std::size_t seed = DEFAULT_SEED) const;
    };
} // namespace compg