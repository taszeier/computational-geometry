#pragma once
#include "data_structures/DoublyConnectedEdgeList.hpp"

namespace compg {
    /**
     * @brief A region of the plane defined by a subdivision and a set of its faces.
     */
    struct PlaneRegion {
        DoublyConnectedEdgeList EdgeList;
        std::unordered_set<DoublyConnectedEdgeList::face_index> FaceIndices;
    };
} // namespace compg