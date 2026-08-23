#pragma once

#include "common/Vertex.hpp"
#include "math/Geometry.hpp"
#include "math/primitives/Polygon.hpp"

namespace compg {
    inline auto GetVertices(const std::vector<Polygon>& polygons) {
        std::vector<Vertex2D> vertices;
        const auto numVertices = std::ranges::fold_left(
            polygons | std::views::transform([](const auto& p) { return p.NumVertices(); }), 0UZ, std::plus{}
        );
        vertices.reserve(numVertices);

        std::ranges::for_each(polygons, [&vertices](const Polygon& polygon) {
            std::ranges::for_each(std::views::iota(0UZ, polygon.NumVertices()), [&polygon, &vertices](const auto i) {
                vertices.emplace_back(polygon.GetVertex(i));
            });
        });

        return vertices;
    }

    /**
     * @brief Determine whether the line segment from vertex to v1 is inside the polygon locally at v1.
     */
    inline bool
    EntersPolygon(const Vertex2D& vertex, const Vertex2D& v0, const Vertex2D& v1, const Vertex2D& v2, Float epsilon) {
        const auto vertexAngle = Angle360(v2 - v1, vertex - v1);
        const auto prevAngle = Angle360(v2 - v1, v0 - v1);
        return epsilon < vertexAngle && vertexAngle + epsilon < prevAngle;
    }
} // namespace compg