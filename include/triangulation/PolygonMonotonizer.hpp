#pragma once

#include "data_structures/DoublyConnectedEdgeList.hpp"
#include "triangulation/PolygonMonotonizerResult.hpp"

namespace compg {

    class PolygonMonotonizer {
    public:
        /**
         * @brief Make a polygon y-monotone.
         * @param polygon A polygon represented as a doubly connected edge list.
         * @return The PolygonMonotonizerResult.
         */
        PolygonMonotonizerResult MakeMonotone(DoublyConnectedEdgeList& polygon) const;
    };
} // namespace compg