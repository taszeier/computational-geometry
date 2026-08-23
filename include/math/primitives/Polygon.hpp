#pragma once

#include "common/Vertex.hpp"
#include "math/primitives/LineSegment.hpp"

namespace compg {
    /**
     * @brief A polygon defined by its vertices in counter-clockwise order.
     */
    class Polygon {
    public:
        explicit Polygon(const std::vector<Vertex2D>& vertices)
            : Vertices(vertices) {
            COMPG_ASSERT(Vertices.size() >= 3, "Expected at least three vertices");
        }

        template <std::size_t N>
        explicit Polygon(const std::array<Vertex2D, N>& vertices)
            : Vertices(vertices.begin(), vertices.end()) {
            COMPG_ASSERT(Vertices.size() >= 3, "Expected at least three vertices");
        }

        [[nodiscard]] const auto& GetVertices() const {
            return Vertices;
        }

        [[nodiscard]] auto NumVertices() const {
            return Vertices.size();
        }

        [[nodiscard]] const auto& GetVertex(std::size_t i) const {
            return Vertices.at(i);
        }

        [[nodiscard]] auto NumEdges() const {
            return Vertices.size();
        }

        [[nodiscard]] LineSegment2D GetEdge(std::size_t i) const {
            COMPG_ASSERT(i < NumEdges(), CreateOutOfRangeMessage("i", i, Vertices.size()));
            const auto j = (i + 1) % Vertices.size();
            return {Vertices.at(i), Vertices.at(j)};
        }

    private:
        std::vector<Vertex2D> Vertices;
    };

    /**
     * @brief Remove vertices such that the polygon represents the same shape.
     * @param polygon A polygon.
     * @return The optimized polygon.
     */
    Polygon Optimize(const Polygon& polygon);

    /**
     * @brief Determine whether polygons are equal regardless of the order of the vertices.
     * @param polygon1 A polygon.
     * @param polygon2 A polygon.
     * @param epsilon An error threshold used to compare vertices.
     * @return Whether the polygons are equal.
     */
    bool AreEqual(const Polygon& polygon1, const Polygon& polygon2, Float epsilon = EPSILON);
} // namespace compg