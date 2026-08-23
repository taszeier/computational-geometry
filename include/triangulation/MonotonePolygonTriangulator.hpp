#pragma once
#include "data_structures/DoublyConnectedEdgeList.hpp"

namespace compg {

    class MonotonePolygonTriangulator {
    public:
        /**
         * @brief Triangulate a y-monotone subpolygon.
         * @param polygon A doubly connected edge list representing the polygon.
         * @param edgeIndex An edge index on the boundary of a y-monotone subpolygon.
         */
        void Triangulate(DoublyConnectedEdgeList& polygon, DoublyConnectedEdgeList::edge_index edgeIndex) const;
    };
} // namespace compg
