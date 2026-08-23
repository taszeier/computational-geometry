#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <variant>

#include "common/Vertex.hpp"
#include "data_structures/graph/DirectedEdge.hpp"
#include "math/Geometry.hpp"

namespace compg {
    struct Minus1 {};
    struct Minus2 {};

    struct VertexIndex {
        std::variant<Minus2, Minus1, std::size_t> Index;

        constexpr bool operator<(const VertexIndex& other) const {
            return Index.index() < other.Index.index()
                   || (Index.index() == other.Index.index() && Index.index() == 2UZ
                       && std::get<2>(Index) < std::get<2>(other.Index));
        }

        constexpr bool operator==(const VertexIndex& other) const {
            return !(*this < other) && !(other < *this);
        }
    };

    struct IndexTriangle {
        using indices_type = std::array<VertexIndex, 3>;

        constexpr explicit IndexTriangle(const std::array<VertexIndex, 3>& indices)
            : Indices(indices) {
            RotateIndices();
        }

        constexpr IndexTriangle(VertexIndex v0, VertexIndex v1, VertexIndex v2)
            : Indices{v0, v1, v2} {
            RotateIndices();
        }

        constexpr bool operator==(const IndexTriangle& other) const {
            return Indices.at(0) == other.Indices.at(0) && Indices.at(1) == other.Indices.at(1)
                   && Indices.at(2) == other.Indices.at(2);
        }

        constexpr auto operator[](std::size_t i) const {
            return Indices.at(i);
        }

    private:
        constexpr void RotateIndices() {
            const auto it = std::ranges::min_element(Indices, std::less<VertexIndex>{});
            std::ranges::rotate(Indices, it);
        }

    public:
        indices_type Indices;
    };

    struct DelaunayLess {
        static constexpr auto operator()(const auto& v1, const auto& v2) {
            return v1[1] < v2[1] || (v1[1] == v2[1] && v1[0] < v2[0]);
        }

        static constexpr auto Compare(const auto& v1, const auto& v2) {
            return operator()(v1, v2);
        }
    };

    /**
     * @param v0 An index of a vertex.
     * @param v1 An index of a vertex.
     * @param vertex The query point.
     * @param vertexLocator A callable object that returns the location of a vertex with a non-negative index.
     * @return Whether the query point is to the left of the line from v0 to v1 or on it.
     */
    constexpr bool NotRightOf(VertexIndex v0, VertexIndex v1, const Vertex2D& vertex, const auto& vertexLocator) {
        if (v0.Index.index() != 2 && v1.Index.index() != 2) {
            return v0.Index.index() == 0 && v1.Index.index() == 1;
        }

        if (v0.Index.index() == 2 && v1.Index.index() == 2) {
            const auto i0 = std::get<2>(v0.Index);
            const auto i1 = std::get<2>(v1.Index);
            return FindSide(vertexLocator(i0), vertexLocator(i1), vertex) != PointSide::Positive;
        }

        if (v0.Index.index() == 2) {
            const auto i0 = std::get<2>(v0.Index);
            if (v1.Index.index() == 1) {
                return DelaunayLess::Compare(vertexLocator(i0), vertex);
            }
            return DelaunayLess::Compare(vertex, vertexLocator(i0));
        }
        // v1.Index.index() == 2
        const auto i1 = std::get<2>(v1.Index);
        if (v0.Index.index() == 0) {
            return DelaunayLess::Compare(vertexLocator(i1), vertex);
        }
        return DelaunayLess::Compare(vertex, vertexLocator(i1));
    }

    /**
     * @param triangle A triangle in the triangulation.
     * @param vertex The query point.
     * @param vertexLocator A callable object that returns the location of a vertex with a non-negative index.
     * @return Whether the triangle contains the query point.
     */
    bool Contains(const IndexTriangle& triangle, const Vertex2D& vertex, const auto& vertexLocator) {
        return NotRightOf(triangle[0], triangle[1], vertex, vertexLocator)
               && NotRightOf(triangle[1], triangle[2], vertex, vertexLocator)
               && NotRightOf(triangle[2], triangle[0], vertex, vertexLocator);
    }

    struct AdjacentTriangles {
        DirectedEdge<VertexIndex> CommonEdge;
        VertexIndex LeftVertex;
        VertexIndex RightVertex;

        [[nodiscard]] constexpr auto GetTriangles() const {
            const IndexTriangle tri0{CommonEdge[0], CommonEdge[1], LeftVertex};
            const IndexTriangle tri1{CommonEdge[1], CommonEdge[0], RightVertex};

            return std::tuple{tri0, tri1};
        }
    };

} // namespace compg

namespace std {
    template <>
    struct hash<compg::VertexIndex> {
        size_t operator()(const compg::VertexIndex& index) const noexcept {
            size_t seed{0};
            visit(
                compg::Overloads{
                    [&seed](compg::Minus2) { compg::hash_combine(seed, -2); },
                    [&seed](compg::Minus1) { compg::hash_combine(seed, -1); },
                    [&seed](std::size_t i) { compg::hash_combine(seed, i); }
                },
                index.Index
            );
            return seed;
        }
    };

    template <>
    struct hash<compg::IndexTriangle> {
        size_t operator()(const compg::IndexTriangle& triangle) const noexcept {
            return hash<typename compg::IndexTriangle::indices_type>()(triangle.Indices);
        }
    };
} // namespace std