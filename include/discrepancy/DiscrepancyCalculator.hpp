#pragma once

#include "common/Vertex.hpp"

namespace compg {
    class DiscrepancyCalculator {
    public:
        /**
         * @param vertices A vector of unique points in the range [0,1] x [0,1].
         * @return The half plane discrepancy of the vertices.
         */
        [[nodiscard]] Float FindDiscrepancy(const std::vector<Vertex2D>& vertices) const;
    };
} // namespace compg