#pragma once

#include "data_structures/DoublyConnectedEdgeList.hpp"

namespace compg {
    class ArtGalleryProblem {
    public:
        /**
         * @brief Find the minimum number of cameras to surveil the art gallery.
         * @param polygon A simple polygon representing the walls of the art gallery.
         * @return The smallest number of cameras.
         */
        [[nodiscard]] std::size_t FindNumCameras(const DoublyConnectedEdgeList& polygon) const;

        /**
         * @brief Find possible camera arrangements in the art gallery.
         * @param polygon A simple polygon representing the walls of the art gallery.
         * @return A camera index for each vertex of the polygon.
         */
        [[nodiscard]] std::unordered_map<DoublyConnectedEdgeList::vertex_index, std::size_t>
        FindCameraMap(const DoublyConnectedEdgeList& polygon) const;
    };
} // namespace compg