#pragma once

#include "common/Vertex.hpp"
#include "data_structures/DoublyConnectedEdgeList.hpp"
#include "math/primitives/Box.hpp"
#include "voronoi/Common.hpp"

namespace compg {
    class VoronoiCalculator {
    public:
        using event_id_type = std::variant<SiteEventId, CircleEventId>;

        /**
         * @brief Find the Voronoi diagram of a set of 2D vertices.
         * @param sites A vector of unique points in the plane.
         * @param box The region of the plane used to crop the infinite edges.
         * @return The region of the Voronoi diagram inside the box.
         */
        [[nodiscard]] DoublyConnectedEdgeList
        FindVoronoiDiagram(const std::vector<Vertex2D>& sites, const Box2D& box) const;
    };
} // namespace compg
