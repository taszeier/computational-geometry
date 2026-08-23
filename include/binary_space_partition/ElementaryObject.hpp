#pragma once

#include "math/primitives/LineSegment.hpp"
#include "math/primitives/Triangle.hpp"

namespace compg {
    template <std::size_t K>
    struct ElementaryObject;

    template <>
    struct ElementaryObject<2UZ> {
        LineSegment2D Segment;
    };

    using ElementaryObject2D = ElementaryObject<2UZ>;

    template <>
    struct ElementaryObject<3UZ> {
        Triangle3D Triangle;
    };

    using ElementaryObject3D = ElementaryObject<3UZ>;
} // namespace compg
