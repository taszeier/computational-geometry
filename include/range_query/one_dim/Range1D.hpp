#pragma once

#include "common/Common.hpp"

namespace compg {
    template <typename T>
    struct Range1D {
        constexpr Range1D(T lower, T upper)
            : Lower(lower)
            , Upper(upper) {
            COMPG_ASSERT(lower <= upper, "Received invalid range parameters");
        }

        constexpr bool Contains(T value) const {
            return Lower <= value && value <= Upper;
        }

        T Lower;
        T Upper;
    };

} // namespace compg