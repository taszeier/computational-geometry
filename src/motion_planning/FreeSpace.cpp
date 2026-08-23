#include "motion_planning/FreeSpace.hpp"

namespace compg {
    bool Contains(const FreeSpace& freeSpace, const Vertex2D& vertex) {
        if (!freeSpace.Box.Contains(vertex)) {
            return false;
        }

        const auto searchResult = freeSpace.SearchStructure.Search(vertex);
        if (searchResult.State.index() == 0 && freeSpace.ObstacleTrapezoids.contains(std::get<0>(searchResult.State))) {
            return false;
        }
        return true;
    }
} // namespace compg