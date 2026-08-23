#pragma once

#include "common/Common.hpp"
#include "common/Vertex.hpp"

#include <cmath>

namespace compg::math {
    static constexpr Float PI = 3.14159265358979323846;

    constexpr Float Determinant(const Vertex2D& v1, const Vertex2D& v2) {
        return v1[0] * v2[1] - v1[1] * v2[0];
    }

    constexpr bool IsZero(Float x, Float epsilon = EPSILON) {
        return std::abs(x) <= epsilon;
    }

    template <typename V>
    constexpr Float NormSquared(const V& v) {
        return v.squaredNorm();
    }

    template <typename V>
    constexpr Float Norm(const V& v) {
        return std::sqrt(NormSquared(v));
    }

    template <std::size_t K>
    constexpr Float InfinityNorm(const Vertex<K>& v) {
        return v.template lpNorm<Eigen::Infinity>();
    }

    template <typename V>
    constexpr bool IsZero(const V& v, Float epsilon = EPSILON) {
        return NormSquared(v) <= epsilon * epsilon;
    }

    template <typename V>
    constexpr Float Dot(const V& v1, const V& v2) {
        return v1.dot(v2);
    }

    template <typename V>
    constexpr std::enable_if_t<std::is_same_v<V, Vertex2D>, Float> Cross(const Vertex2D& v1, const Vertex2D& v2) {
        return Determinant(v1, v2);
    }

    inline Vertex3D Cross(const Vertex3D& a, const Vertex3D& b, const Vertex3D& c) {
        const Vertex3D ab = b - a;
        const Vertex3D ac = c - a;
        const auto x = ab[1] * ac[2] - ab[2] * ac[1];
        const auto y = ab[2] * ac[0] - ab[0] * ac[2];
        const auto z = ab[0] * ac[1] - ab[1] * ac[0];

        return Vertex3D{x, y, z};
    }

    template <typename V>
    constexpr V LinearInterpolate(const V& v1, const V& v2, Float t) {
        return v1 * (1 - t) + t * v2;
    }
} // namespace compg::math

namespace compg {
    template <typename V>
    bool AreEqual(const V& v1, const V& v2, Float epsilon = EPSILON) {
        return math::NormSquared(V{v1 - v2}) <= epsilon * epsilon;
    }

    template <typename V>
    bool AreEqual(const std::vector<V>& vertices1, const std::vector<V>& vertices2, Float epsilon = EPSILON) {
        if (vertices1.size() != vertices2.size()) {
            return false;
        }

        return std::ranges::all_of(std::views::zip(vertices1, vertices2), [epsilon](const auto& tup) {
            const auto& [v1, v2] = tup;
            return AreEqual(v1, v2, epsilon);
        });
    }
} // namespace compg