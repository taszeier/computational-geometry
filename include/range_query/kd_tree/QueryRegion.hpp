#pragma once

#include "range_query/kd_tree/KdTreeRegion.hpp"

namespace compg {
    template <std::size_t K>
    class QueryRegion {
    public:
        virtual constexpr bool Contains(const Vertex<K>&) const = 0;
        virtual constexpr bool Covers(const KdTreeRegion<K>&) const = 0;
        virtual constexpr bool Intersects(const KdTreeRegion<K>&) const = 0;

        QueryRegion() = default;
        QueryRegion(const QueryRegion&) = delete;
        QueryRegion(QueryRegion&&) = delete;
        QueryRegion& operator=(const QueryRegion&) = delete;
        QueryRegion& operator=(QueryRegion&&) = delete;
        virtual ~QueryRegion() = default;
    };
} // namespace compg
