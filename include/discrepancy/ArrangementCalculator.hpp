#pragma once

#include "data_structures/DoublyConnectedEdgeList.hpp"
#include "math/primitives/Line.hpp"

namespace compg {
    class ArrangementCalculator {
    public:
        /**
         * @param lines A vector of unique lines that are not all parallel.
         * @return A doubly connected edge list of the subdivision induced by the lines. The infinite half edges are
         * bounded by a bounding box.
         */
        [[nodiscard]] DoublyConnectedEdgeList FindArrangement(const std::vector<Line2D>& lines) const;

        struct ArrangementDetails {
            using map_type = std::unordered_map<std::size_t, DoublyConnectedEdgeList::edge_index>;

            DoublyConnectedEdgeList Arrangement;
            map_type FirstEdgeMap;
        };

        /**
         * @param lines A vector of unique lines that are not all parallel.
         * @return A doubly connected edge list of the subdivision induced by the lines and the index of the first half
         * edge for each line. The infinite half edges are bounded by a bounding box and the origin of each first half
         * edge is lexicographically less than the destination of the half edge.
         */
        [[nodiscard]] ArrangementDetails FindArrangementDetails(const std::vector<Line2D>& lines) const;
    };
} // namespace compg