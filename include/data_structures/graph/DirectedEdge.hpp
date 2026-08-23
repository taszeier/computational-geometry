#pragma once

#include "common/Hash.hpp"
#include <array>

namespace compg {
    template <typename VertexType>
    class DirectedEdge {
    public:
        using array_type = std::array<VertexType, 2>;

        constexpr DirectedEdge(const VertexType& v1, const VertexType& v2)
            : Vertices{v1, v2} {}

        constexpr bool operator==(const DirectedEdge& other) const {
            return Vertices[0] == other.Vertices[0] && Vertices[1] == other.Vertices[1];
        }

        constexpr const array_type& GetVertices() const& {
            return Vertices;
        }

        constexpr auto operator[](std::size_t i) const {
            return Vertices.at(i);
        }

        constexpr auto At(std::size_t i) const {
            return Vertices.at(i);
        }

    private:
        array_type Vertices;
    };

    template <typename VertexType>
    constexpr auto make_directed_edge(const VertexType& v1, const VertexType& v2) -> DirectedEdge<VertexType> {
        return DirectedEdge<VertexType>(v1, v2);
    }
} // namespace compg

namespace std {
    template <typename VertexType>
    struct hash<compg::DirectedEdge<VertexType>> {
        size_t operator()(const compg::DirectedEdge<VertexType>& edge) const noexcept {
            return std::hash<typename compg::DirectedEdge<VertexType>::array_type>()(edge.GetVertices());
        }
    };
} // namespace std