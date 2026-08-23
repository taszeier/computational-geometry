#pragma once

#include "common/Common.hpp"
#include "math/Side.hpp"
#include "math/primitives/Hyperplane.hpp"

namespace compg {
    template <typename HyperplaneType>
    class HalfPlane;

    template <std::size_t K>
    class HalfPlane<Hyperplane<K>> {
    public:
        void Flip() {
            Plane.Flip();
        }

        Hyperplane<K> Plane;
    };

    template <std::size_t K>
    class HalfPlane<AxesAlignedHyperplane<K>> {
    public:
        void Flip() {
            NormalSide = (NormalSide == Side::Positive) ? Side::Negative : Side::Positive;
        }

        AxesAlignedHyperplane<K> Plane;
        Side NormalSide = Side::Positive;
    };
} // namespace compg