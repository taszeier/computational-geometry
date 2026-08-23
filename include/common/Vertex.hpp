#pragma once

#include "common/Common.hpp"
#include "common/Hash.hpp"

namespace compg {
    /*template <std::size_t N>
    using Vertex = Eigen::Matrix<Float, N, 1>;*/
    template <std::size_t N>
    struct Vertex : Eigen::Matrix<Float, N, 1> {
        using Eigen::Matrix<Float, N, 1>::Matrix;
    };
    using Vertex2D = Vertex<2UZ>;
    using Vertex3D = Vertex<3UZ>;

    template <std::size_t K, typename BinaryOperationType>
    Vertex<K> ElementWise(const Vertex<K>& v1, const Vertex<K>& v2, BinaryOperationType&& binaryOperation) {
        Vertex<K> result;
        for (std::size_t i : Indices<K>) {
            result(i) = binaryOperation(v1(i), v2(i));
        }
        return result;
    }

    template <std::size_t K>
    Vertex<K> ElementWiseMin(const Vertex<K>& v1, const Vertex<K>& v2) {
        return ElementWise<K>(v1, v2, [](Float a, Float b) { return std::min(a, b); });
    }

    template <std::size_t K>
    Vertex<K> ElementWiseMax(const Vertex<K>& v1, const Vertex<K>& v2) {
        return ElementWise<K>(v1, v2, [](Float a, Float b) { return std::max(a, b); });
    }

    template <std::size_t K>
    constexpr Vertex<K> CreateNullVector() {
        Vertex<K> v;
        for (auto j : Indices<K>) {
            v(j) = 0;
        }
        return v;
    }

    template <std::size_t K>
    constexpr Vertex<K> CreateCanonicalVector(std::size_t i) {
        COMPG_ASSERT(i < K, CreateOutOfRangeMessage("i", i, K));
        auto v = CreateNullVector<K>();
        v(i) = 1;
        return v;
    }

    template <typename V>
    struct LexicographicalLess;

    template <std::size_t K>
    struct LexicographicalLess<Vertex<K>> {
        static bool operator()(const Vertex<K>& lhs, const Vertex<K>& rhs) noexcept {
            for (const auto i : Indices<K>) {
                if (lhs(i) < rhs(i)) {
                    return true;
                }
                if (lhs(i) > rhs(i)) {
                    return false;
                }
            }
            return false;
        }

        static bool Compare(const Vertex<K>& lhs, const Vertex<K>& rhs) noexcept {
            return operator()(lhs, rhs);
        }
    };

    template <std::size_t K>
    class CoordinateProjector {
    public:
        constexpr explicit CoordinateProjector(std::size_t coordinateIndex)
            : CoordinateIndex(coordinateIndex) {
            COMPG_ASSERT(coordinateIndex < K, CreateOutOfRangeMessage("coordinateIndex", coordinateIndex, K));
        }

        constexpr auto operator()(const Vertex<K>& vertex) const {
            return vertex(CoordinateIndex);
        }

    private:
        std::size_t CoordinateIndex;
    };

    template <std::size_t K>
    struct RotatedLexicographicalLess {
        explicit constexpr RotatedLexicographicalLess(std::size_t coordinateIndex)
            : Indices(ConvertToArray(std::make_index_sequence<K>())) {
            COMPG_ASSERT(coordinateIndex < K, CreateOutOfRangeMessage("coordinateIndex", coordinateIndex, K));
            std::ranges::rotate(Indices, std::next(Indices.begin(), coordinateIndex));
        }

        constexpr bool operator()(const Vertex<K>& lhs, const Vertex<K>& rhs) const noexcept {
            for (auto i : Indices) {
                if (lhs(i) < rhs(i)) {
                    return true;
                }
                if (lhs(i) > rhs(i)) {
                    return false;
                }
            }
            return false;
        }
        constexpr bool operator()(const Vertex<K>& lhs, Float rhs) const noexcept {
            return lhs(Indices[0]) < rhs;
        }

        constexpr bool operator()(Float lhs, const Vertex<K>& rhs) const noexcept {
            return lhs < rhs(Indices[0]);
        }

    private:
        std::array<std::size_t, K> Indices;
    };

    template <std::size_t K>
    struct VertexRecord {
        std::size_t Index;
        Vertex<K> Value;
    };

    struct ValueProjector {
        static constexpr auto operator()(const auto& record) noexcept {
            return record.Value;
        }
    };

    struct IndexProjector {
        static constexpr auto operator()(const auto& record) noexcept {
            return record.Index;
        }
    };
} // namespace compg

namespace std {
    template <size_t K>
    struct hash<compg::Vertex<K>> {
        size_t operator()(const compg::Vertex<K>& vertex) const noexcept {
            size_t seed{0};
            std::ranges::for_each(compg::Indices<K>, [&seed, &vertex](auto i) {
                compg::hash_combine(seed, vertex(i));
            });
            return seed;
        }
    };

    template <size_t K>
    struct formatter<compg::Vertex<K>> {
        constexpr auto parse(std::format_parse_context& ctx) {
            return ctx.begin();
        }

        auto format(const compg::Vertex<K>& vertex, std::format_context& ctx) const {
            auto it = std::format_to(ctx.out(), "[{}", vertex(0));
            std::ranges::for_each(compg::Indices<K> | views::drop(1), [&it, &vertex](auto i) {
                it = std::format_to(it, ", {}", vertex(i));
            });
            *it++ = ']';
            return it;
        }
    };
} // namespace std