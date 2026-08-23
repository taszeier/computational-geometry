#pragma once

#include "common/Vertex.hpp"

namespace compg {

    template <std::size_t K>
    class Triangle {
    public:
        using vertex_type = Vertex<K>;
        using array_type = std::array<vertex_type, 3>;

        /**
         * @param vertices The array of vertices in counter-clockwise order.
         */
        explicit Triangle(array_type vertices)
            : Vertices{std::move(vertices)} {}

        /**
         * @brief Initialize the triangle given the vertices in counter-clockwise order.
         * @param a A vertex of the triangle.
         * @param b A vertex of the triangle.
         * @param c A vertex of the triangle.
         */
        Triangle(const vertex_type& a, const vertex_type& b, const vertex_type& c)
            : Vertices{a, b, c} {}

        [[nodiscard]] const auto& GetVertices() const {
            return Vertices;
        }

        const auto& operator[](std::size_t i) const {
            return Vertices.at(i);
        }

        [[nodiscard]] vertex_type Centroid() const {
            constexpr Float oneThird = 1.0 / 3.0;
            return Vertices[0] * oneThird + Vertices[1] * oneThird + Vertices[2] * oneThird;
        }

    private:
        array_type Vertices;
    };

    using Triangle2D = Triangle<2UZ>;
    using Triangle3D = Triangle<3UZ>;

    bool Contains(const Triangle2D& triangle, const Vertex2D& vertex, Float epsilon = EPSILON);
} // namespace compg
