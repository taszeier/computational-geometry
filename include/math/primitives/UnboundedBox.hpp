#pragma once

#include <algorithm>

#include "common/Common.hpp"
#include "math/primitives/HalfPlane.hpp"

namespace compg {
    template <std::size_t K>
    class UnboundedBox {
    public:
        using plane_type = AxesAlignedHyperplane<K>;

        void Intersect(const HalfPlane<plane_type>& halfPlane) {
            Intersect(halfPlane.Plane, halfPlane.NormalSide);
        }

        void Intersect(const plane_type& plane, Side side) {
            switch (side) {
            case Side::Positive:
                IntersectLowerPlane(plane);
                return;
            case Side::Negative:
                IntersectUpperPlane(plane);
                return;
            }
            COMPG_THROW("Received unexpected value for side");
        }

        void Intersect(const UnboundedBox& unboundedBox) {
            std::ranges::for_each(unboundedBox.LowerBounds | std::views::enumerate, [this](const auto& pair) {
                const auto [i, maybeBound] = pair;
                if (maybeBound) {
                    IntersectLowerPlane(plane_type{static_cast<std::size_t>(i), maybeBound.value()});
                }
            });
            std::ranges::for_each(unboundedBox.UpperBounds | std::views::enumerate, [this](const auto& pair) {
                const auto [i, maybeBound] = pair;
                if (maybeBound) {
                    IntersectUpperPlane(plane_type{static_cast<std::size_t>(i), maybeBound.value()});
                }
            });
        }

        template <typename... Args>
        auto Intersected(Args&&... args) const {
            auto copy{*this};
            copy.Intersect(std::forward<Args>(args)...);
            return copy;
        }

        [[nodiscard]] constexpr bool IsBounded() const {
            return IsEmpty() || (HasLowerBound() && HasUpperBound());
        }

        [[nodiscard]] constexpr bool IsEmpty() const {
            return std::ranges::any_of(std::views::zip(LowerBounds, UpperBounds), [](const auto& boundsPair) {
                const auto& [lower, upper] = boundsPair;
                return lower && upper && lower.value() > upper.value();
            });
        }

        [[nodiscard]] constexpr std::optional<Vertex<K>> GetLowerCorner() const {
            return IsEmpty() ? std::nullopt : GetCorner(LowerBounds);
        }

        [[nodiscard]] constexpr std::optional<Vertex<K>> GetUpperCorner() const {
            return IsEmpty() ? std::nullopt : GetCorner(UpperBounds);
        }

        [[nodiscard]] constexpr auto GetCorners() const {
            auto noEmpty = [](const auto& maybeBoundTuple) {
                bool foundEmpty{false};
                for_each_in_tuple(maybeBoundTuple, [&foundEmpty](auto, const auto& maybeBound) {
                    foundEmpty = foundEmpty || !maybeBound.has_value();
                });
                return !foundEmpty;
            };

            auto toVertex = [](const auto& maybeBoundTuple) {
                Vertex<K> result;
                for_each_in_tuple(maybeBoundTuple, [&result](auto i, const auto& maybeBound) {
                    result(i) = maybeBound.value();
                });
                return result;
            };

            auto corners = std::make_optional(
                CreatePlanesCartesianProduct(std::make_index_sequence<K>()) | std::views::filter(noEmpty)
                | std::views::transform(toVertex)
            );
            if (IsEmpty()) {
                corners.reset();
            }
            return corners;
        }

        [[nodiscard]] constexpr auto GetLowerPlanes() const {
            auto planes = std::make_optional(GetPlanes(LowerBounds));
            if (IsEmpty()) {
                planes.reset();
            }
            return planes;
        }

        [[nodiscard]] constexpr auto GetUpperPlanes() const {
            auto planes = std::make_optional(GetPlanes(UpperBounds));
            if (IsEmpty()) {
                planes.reset();
            }
            return planes;
        }

    private:
        [[nodiscard]] constexpr bool HasLowerBound() const {
            return std::ranges::all_of(LowerBounds, &std::optional<Float>::has_value);
        }

        [[nodiscard]] constexpr bool HasUpperBound() const {
            return std::ranges::all_of(UpperBounds, &std::optional<Float>::has_value);
        }

        [[nodiscard]] constexpr auto GetPlanes(const auto& bounds) const {
            auto isBounded = [](const auto pair) {
                const auto [i, maybeBound] = pair;
                return maybeBound.has_value();
            };
            auto toPlane = [](const auto pair) {
                const auto [i, maybeBound] = pair;
                return plane_type{static_cast<std::size_t>(i), maybeBound.value()};
            };
            return bounds | std::views::enumerate | std::views::filter(isBounded) | std::views::transform(toPlane);
        }

        template <std::size_t... Is>
        constexpr auto CreatePlanesCartesianProduct(std::index_sequence<Is...>) const {
            return std::views::cartesian_product(std::array{LowerBounds.at(Is), UpperBounds.at(Is)}...);
        }

        void IntersectLowerPlane(const plane_type& plane) {
            const auto i = plane.GetAxisIndex();
            LowerBounds.at(i)
                = LowerBounds.at(i)
                      .transform([&plane](Float bound) { return std::max(plane.GetIntersection(), bound); })
                      .value_or(plane.GetIntersection());
        }

        void IntersectUpperPlane(const plane_type& plane) {
            const auto i = plane.GetAxisIndex();
            UpperBounds.at(i)
                = UpperBounds.at(i)
                      .transform([&plane](Float bound) { return std::min(plane.GetIntersection(), bound); })
                      .value_or(plane.GetIntersection());
        }

        [[nodiscard]] constexpr std::optional<Vertex<K>> GetCorner(const auto& bounds) const {
            Vertex<K> corner;
            for (auto [i, maybeBound] : std::views::enumerate(bounds)) {
                if (maybeBound) {
                    corner(i) = maybeBound.value();
                } else {
                    return std::nullopt;
                }
            }
            return corner;
        }

    private:
        std::array<std::optional<Float>, K> LowerBounds;
        std::array<std::optional<Float>, K> UpperBounds;
    };
} // namespace compg