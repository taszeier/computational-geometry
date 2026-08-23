#pragma once
#include "data_structures/DoublyConnectedEdgeList.hpp"

namespace compg {
    /**
     * @brief Overlay the edge lists and keep the faces that are bounded in either.
     * @details The faces must be valid in both input edge lists.
     * @param edgeList1 A doubly connected edge list.
     * @param edgeList2 A doubly connected edge list.
     * @return The union.
     */
    DoublyConnectedEdgeList Union(const DoublyConnectedEdgeList& edgeList1, const DoublyConnectedEdgeList& edgeList2);

    /**
     * @brief Overlay the edge lists and keep the faces that are bounded in both.
     * @details The faces must be valid in both input edge lists.
     * @param edgeList1 A doubly connected edge list.
     * @param edgeList2 A doubly connected edge list.
     * @return The intersection.
     */
    DoublyConnectedEdgeList
    Intersection(const DoublyConnectedEdgeList& edgeList1, const DoublyConnectedEdgeList& edgeList2);

    /**
     * @brief Overlay the edge lists and keep the faces that are bounded in the first edge list but not in the second.
     * @details The faces must be valid in both input edge lists.
     * @param edgeList1 A doubly connected edge list.
     * @param edgeList2 A doubly connected edge list.
     * @return The difference.
     */
    DoublyConnectedEdgeList
    Difference(const DoublyConnectedEdgeList& edgeList1, const DoublyConnectedEdgeList& edgeList2);
} // namespace compg
