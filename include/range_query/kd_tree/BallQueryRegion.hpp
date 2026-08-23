#pragma once

#include "math/Conversions.hpp"
#include "math/Geometry.hpp"
#include "math/primitives/Ball.hpp"
#include "range_query/kd_tree/QueryRegion.hpp"

namespace compg {
    template <std::size_t K>
    class BallQueryRegion : public QueryRegion<K> {
    public:
        explicit BallQueryRegion(const Ball<K>& ball)
            : Ball(ball) {}

        [[nodiscard]] constexpr bool Covers(const KdTreeRegion<K>& region) const override {
            if (!region.IsBounded()) {
                return false;
            }

            return region.GetCorners()
                .transform([this](auto&& corners) {
                    return std::ranges::all_of(corners, [this](const Vertex<K>& corner) {
                        return Ball.Contains(corner);
                    });
                })
                .value_or(true);
        }

        [[nodiscard]] constexpr bool Intersects(const KdTreeRegion<K>& region) const override {
            if (region.IsEmpty()) {
                return false;
            }
            auto lowerPlanes = region.GetLowerPlanes();
            auto lowerHalfPlanes = lowerPlanes.value() | std::views::transform([](const auto& plane) {
                                       return HalfPlane<AxesAlignedHyperplane<K>>{plane, Side::Positive};
                                   });
            auto upperPlanes = region.GetUpperPlanes();
            auto upperHalfPlanes = upperPlanes.value() | std::views::transform([](const auto& plane) {
                                       return HalfPlane<AxesAlignedHyperplane<K>>{plane, Side::Negative};
                                   });
            auto isBehind = [this](const auto& halfPlane) {
                return FindHalfPlaneRelation(halfPlane, Ball) == HalfPlaneRelation::Negative;
            };

            return std::ranges::none_of(lowerHalfPlanes, isBehind) && std::ranges::none_of(upperHalfPlanes, isBehind);
        }

        [[nodiscard]] constexpr bool Contains(const Vertex<K>& vertex) const override {
            return Ball.Contains(vertex);
        }

    private:
        Ball<K> Ball;
    };
} // namespace compg