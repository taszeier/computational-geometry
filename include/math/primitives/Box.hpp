#pragma once

#include <functional>
#include <numeric>
#include <ranges>

#include "common/Common.hpp"
#include "common/Error.hpp"
#include "common/Vertex.hpp"
#include "math/primitives/Interval.hpp"
#include "math/primitives/LineSegment.hpp"
#include "math/primitives/Triangle.hpp"

namespace compg {

    template <std::size_t N>
    class Box {
    public:
        static constexpr std::size_t K = N;
        using vertex_type = Vertex<N>;

    public:
        constexpr Box(const vertex_type& v1, const vertex_type& v2) {
            SetCorners(v1, v2);
        }

        template <typename... Args>
        constexpr explicit Box(const Interval<Args>&... args) {
            static_assert(sizeof...(Args) == N, "Invalid number of arguments");
            static_assert(
                (std::conjunction_v<std::is_convertible<Args, Float>> && ...),
                "Cannot convert value type of interval to Float"
            );

            const Vertex<N> lower{static_cast<Float>(args.Lower)...};
            const Vertex<N> upper{static_cast<Float>(args.Upper)...};
            SetCorners(lower, upper);
        }

        [[nodiscard]] constexpr bool IsEmpty() const {
            return std::ranges::any_of(Indices, [this](auto i) { return LowerCorner(i) == UpperCorner(i); });
        }

        [[nodiscard]] constexpr bool Contains(const Vertex<N>& v) const {
            return std::ranges::all_of(Indices, [this, &v](auto i) {
                return LowerCorner(i) <= v(i) && v(i) <= UpperCorner(i);
            });
        }

        [[nodiscard]] constexpr vertex_type GetCenter() const {
            return (GetLowerCorner() + GetUpperCorner()) * 0.5;
        }

        [[nodiscard]] constexpr Float GetSideLength(std::size_t i) const {
            COMPG_ASSERT(i < N, CreateOutOfRangeMessage("i", i, N));
            return GetUpperCorner()(i) - GetLowerCorner()(i);
        }

        [[nodiscard]] constexpr Float Size() const {
            return std::ranges::fold_left(
                std::views::zip_transform(std::minus<Float>(), UpperCorner, LowerCorner), static_cast<Float>(1),
                std::multiplies<Float>()
            );
        }

        const vertex_type& GetLowerCorner() const {
            return LowerCorner;
        }
        const vertex_type& GetUpperCorner() const {
            return UpperCorner;
        }

        // TODO: return std::array not std::vector
        [[nodiscard]] constexpr auto GetCorners() const {
            auto toVertex = [](const auto& boundTuple) {
                Vertex<K> result;
                for_each_in_tuple(boundTuple, [&result](auto i, auto bound) { result(i) = bound; });
                return result;
            };

            return CreatePlanesCartesianProduct(std::make_index_sequence<K>()) | std::views::transform(toVertex)
                   | std::ranges::to<std::vector>();
        }

    private:
        template <std::size_t... Is>
        constexpr auto CreatePlanesCartesianProduct(std::index_sequence<Is...>) const {
            return std::views::cartesian_product(std::array{LowerCorner[Is], UpperCorner[Is]}...);
        }

        void SetCorners(const vertex_type& v1, const vertex_type& v2) {
            std::ranges::for_each(Indices, [this, &v1, &v2](std::size_t i) {
                LowerCorner(i) = std::min(v1(i), v2(i));
                UpperCorner(i) = std::max(v1(i), v2(i));
            });
        }

    private:
        vertex_type LowerCorner;
        vertex_type UpperCorner;

        static constexpr std::array<std::size_t, N> Indices = ConvertToArray(std::make_index_sequence<N>());
    };

    using Box2D = Box<2>;
    using Box3D = Box<3>;

    template <std::size_t K>
    Box<K> Pad(const Box<K>& box, const Vertex<K>& padding) {
        return Box<K>{box.GetLowerCorner() - padding, box.GetUpperCorner() + padding};
    }

    inline Box2D Pad(const Box2D& box, Float xPadding, Float yPadding) {
        return Pad<2UZ>(box, {xPadding, yPadding});
    }

    inline Box3D Pad(const Box3D& box, Float xPadding, Float yPadding, Float zPadding) {
        return Pad<3UZ>(box, {xPadding, yPadding, zPadding});
    }

    inline auto GetCorners(const Box2D& box) {
        const Vertex2D& v0 = box.GetLowerCorner();
        const Vertex2D& v2 = box.GetUpperCorner();
        const Vertex2D v1{v2.x(), v0.y()};
        const Vertex2D v3{v0.x(), v2.y()};

        return std::array{v0, v1, v2, v3};
    }
} // namespace compg
