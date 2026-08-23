#pragma once

#include "common/Random.hpp"
#include "data_structures/DoublyConnectedEdgeList.hpp"

namespace compg {

    class DelaunayTriangulator {
    public:
        /**
         * @param vertices A vector of unique points in the plane.
         * @param seed A seed used to shuffle the vertices.
         * @return A Delaunay triangulation of the vertices.
         */
        [[nodiscard]] DoublyConnectedEdgeList
        Triangulate(const std::vector<Vertex2D>& vertices, std::size_t seed = DEFAULT_SEED) const;
    };
} // namespace compg