#pragma once

#include "common/Common.hpp"
#include "math/primitives/Interval.hpp"

namespace compg {
    struct SegmentQueryRegion {
        Interval<Float> Interval;
        Float Location;
    };
} // namespace compg