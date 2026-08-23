#pragma once

#include "triangulation/MonotonePolygonTriangulator.hpp"
#include "triangulation/PolygonMonotonizer.hpp"

namespace compg {
    class PolygonTriangulator {
    public:
        /**
         * @brief Triangulate a simple polygon.
         * @param polygon A simple polygon.
         */
        void Triangulate(DoublyConnectedEdgeList& polygon) const;

    private:
        PolygonMonotonizer Monotonizer;
        MonotonePolygonTriangulator Triangulator;
    };
} // namespace compg