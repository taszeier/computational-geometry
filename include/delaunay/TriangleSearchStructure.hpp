#pragma once

#include "delaunay/IndexTriangle.hpp"

#include <array>
#include <optional>
#include <unordered_map>
#include <vector>

#include "data_structures/graph/UndirectedEdge.hpp"

namespace compg {

    struct TriangleSearchStructureNode {
        IndexTriangle Triangle;
        std::array<std::optional<std::size_t>, 3> Children{};

        [[nodiscard]] constexpr bool IsLeaf() const {
            return std::ranges::none_of(Children, [](const auto& c) { return c.has_value(); });
        }
    };

    /**
     * @brief A data structure to efficiently find a triangle in the current triangulation that contains a given vertex.
     */
    class TriangleSearchStructure {
    public:
        using edge_type = UndirectedEdge<VertexIndex>;

        explicit TriangleSearchStructure(const IndexTriangle& rootTriangle);

        /**
         * @param vertex The query point.
         * @param vertexLocator A callable object that returns the location of a vertex with a non-negative index.
         * @return A triangle in the current triangulation that contains the query point.
         */
        IndexTriangle Search(const Vertex2D& vertex, const auto& vertexLocator) const {
            auto node = Root;
            while (!Nodes.at(node).IsLeaf()) {
                const auto it
                    = std::ranges::find_if(Nodes.at(node).Children, [this, &vertex, &vertexLocator](const auto mc) {
                          return mc.has_value() && Contains(Nodes.at(mc.value()).Triangle, vertex, vertexLocator);
                      });

                COMPG_ASSERT(it != Nodes.at(node).Children.end(), "Expected at least one child to contain the vertex");
                node = it->value();
            }

            return Nodes.at(node).Triangle;
        }

        /**
         * @brief Update the search structure after a vertex is inserted in the interior of a triangle.
         * @param triangle The triangle that contains the inserted vertex in its interior.
         * @param vertex The index of the inserted vertex.
         */
        void OnCenterSplit(const IndexTriangle& triangle, VertexIndex vertex);
        /**
         * @brief Update the search structure after a vertex is inserted on the edge of a triangle.
         * @param triangles The adjacent triangles that contain the inserted vertex on their common edge.
         * @param vertex The index of the inserted vertex.
         */
        void OnEdgeSplit(const AdjacentTriangles& triangles, VertexIndex vertex);

        /**
         * @brief Update the search structure after an edge is flipped.
         * @param triangles The adjacent triangles whose common edge is flipped.
         */
        void OnEdgeFlip(const AdjacentTriangles& triangles);

    private:
        std::size_t Root;
        std::vector<TriangleSearchStructureNode> Nodes;
        std::unordered_map<IndexTriangle, std::size_t> NodeMap;
    };
} // namespace compg
