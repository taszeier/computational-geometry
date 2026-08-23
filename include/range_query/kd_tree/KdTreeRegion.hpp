#pragma once
#include "math/primitives/UnboundedBox.hpp"

namespace compg {
    template <std::size_t K>
    using KdTreeRegion = UnboundedBox<K>;
}
