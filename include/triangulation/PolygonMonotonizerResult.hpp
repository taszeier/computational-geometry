#pragma once

#include "data_structures/DoublyConnectedEdgeList.hpp"
#include <vector>

namespace compg {
    /**
     * @brief The result of the polygon monotonizer.
     * @details Contains the vector of edge indices that were inserted, the index of the top vertex
     * (where the algorithm starts) and the index of the edge left of the start vertex.
     */
    struct PolygonMonotonizerResult {
        std::vector<DoublyConnectedEdgeList::edge_index> InsertedEdges;
        DoublyConnectedEdgeList::vertex_index BeginVertex;
        DoublyConnectedEdgeList::edge_index BeginLeftEdge;
    };
} // namespace compg