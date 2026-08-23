#pragma once

#include "math/primitives/Box.hpp"

namespace compg {
    template <std::size_t K>
    struct RangeTreeQueryRegion {
        Box<K> Box;

        [[nodiscard]] constexpr bool Contains(const Vertex<K>& vertex) const {
            return Box.Contains(vertex);
        }

        const auto& GetLowerCorner() const {
            return Box.GetLowerCorner();
        }
        const auto& GetUpperCorner() const {
            return Box.GetUpperCorner();
        }
    };
} // namespace compg