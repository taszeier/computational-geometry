#pragma once

#include "common/Random.hpp"
#include "data_structures/DoublyConnectedEdgeList.hpp"
#include "point_location/TrapezoidalMap.hpp"
#include "point_location/TrapezoidalSearchStructure.hpp"

namespace compg {
    /**
     * @brief A data structure for efficiently locating query vertices in a doubly connected edge list.
     */
    class PointLocator {
    public:
        /**
         *
         * @param edgeList The edge list in which queries will be performed.
         * @param seed The seed used to improve average running time.
         */
        explicit PointLocator(const DoublyConnectedEdgeList& edgeList, std::size_t seed = DEFAULT_SEED);

        struct Location {
            using state_type = std::variant<
                DoublyConnectedEdgeList::face_index, DoublyConnectedEdgeList::edge_index,
                DoublyConnectedEdgeList::vertex_index>;
            state_type State;
        };

        /**
         * @brief Locate a point in the doubly connected edge list.
         * @param point The query point.
         * @return The location of the query point. Either a face, edge or vertex of the doubly connected edge list.
         */
        Location LocatePoint(const Vertex2D& point) const;

    private:
        PointLocator(
            const DoublyConnectedEdgeList& edgeList, const std::vector<LineSegment2D>& segments,
            std::size_t seed = DEFAULT_SEED
        );

    private:
        TrapezoidalMap Map;
        TrapezoidalSearchStructure SearchStructure;
        std::unordered_map<LineSegment2D, DoublyConnectedEdgeList::edge_index> EdgeMap;
        DoublyConnectedEdgeList EdgeList;
    };
} // namespace compg