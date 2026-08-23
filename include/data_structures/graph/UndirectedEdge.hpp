#pragma once

#include "common/Hash.hpp"

namespace compg {
    template <typename VertexType, typename ComparatorType = Less>
    class UndirectedEdge {
    public:
        using array_type = std::array<VertexType, 2>;

        constexpr UndirectedEdge(const VertexType& v1, const VertexType& v2, ComparatorType comparator = {})
            : Vertices{Min(v1, v2, comparator), Max(v1, v2, comparator)} {}

        constexpr bool operator==(const UndirectedEdge& other) const {
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
} // namespace compg

namespace std {
    template <typename VertexType, typename ComparatorType>
    struct hash<compg::UndirectedEdge<VertexType, ComparatorType>> {
        size_t operator()(const compg::UndirectedEdge<VertexType, ComparatorType>& edge) const noexcept {
            return std::hash<typename compg::UndirectedEdge<VertexType>::array_type>()(edge.GetVertices());
        }
    };
} // namespace std