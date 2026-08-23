#pragma once

#include "math/Conversions.hpp"
#include "range_query/kd_tree/QueryRegion.hpp"

namespace compg {
    template <std::size_t K>
    class OrthogonalQueryRegion : public QueryRegion<K> {
    public:
        explicit OrthogonalQueryRegion(const Box<K>& box)
            : Box(box) {}

        [[nodiscard]] constexpr bool Covers(const KdTreeRegion<K>& region) const override {
            if (region.IsEmpty()) {
                return true;
            }
            const auto lowerCorner = region.GetLowerCorner();
            const auto upperCorner = region.GetUpperCorner();
            if (lowerCorner && upperCorner) {
                return Box.Contains(lowerCorner.value()) && Box.Contains(upperCorner.value());
            }
            return false;
        }

        [[nodiscard]] constexpr bool Intersects(const KdTreeRegion<K>& region) const override {
            auto box = ConvertTo<KdTreeRegion>(Box);
            box.Intersect(region);
            return !box.IsEmpty();
        }

        [[nodiscard]] constexpr bool Contains(const Vertex<K>& vertex) const override {
            return Box.Contains(vertex);
        }

    private:
        Box<K> Box;
    };
} // namespace compg