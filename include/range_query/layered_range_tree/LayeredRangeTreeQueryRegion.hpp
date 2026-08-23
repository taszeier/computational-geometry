#pragma once

#include "range_query/range_tree/RangeTreeQueryRegion.hpp"

namespace compg {
    template <std::size_t K>
    using LayeredRangeTreeQueryRegion = RangeTreeQueryRegion<K>;
}