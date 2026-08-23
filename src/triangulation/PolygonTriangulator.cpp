#include "triangulation/PolygonTriangulator.hpp"

#include <unordered_set>

namespace compg {
    using edge_index = DoublyConnectedEdgeList::edge_index;
    namespace details {
        std::vector<edge_index>
        FindMonotonePolygonsEdges(const DoublyConnectedEdgeList& polygons, const PolygonMonotonizerResult& result) {
            if (result.InsertedEdges.empty()) {
                return {result.BeginLeftEdge};
            }
            std::unordered_set<edge_index> candidates;
            std::ranges::for_each(result.InsertedEdges, [&candidates, &polygons](edge_index e) {
                candidates.insert(e);
                candidates.insert(polygons.GetTwinIndex(e));
            });

            std::vector<edge_index> polygonHalfEdges;
            polygonHalfEdges.reserve(result.InsertedEdges.size() + 1);
            while (!candidates.empty()) {
                const auto startEdgeIndex = *candidates.begin();
                candidates.erase(candidates.begin());
                polygonHalfEdges.push_back(startEdgeIndex);

                auto currentEdgeIndex = startEdgeIndex;
                do {
                    if (candidates.contains(currentEdgeIndex)) {
                        candidates.erase(currentEdgeIndex);
                    }
                    currentEdgeIndex = polygons.GetNextIndex(currentEdgeIndex);
                } while (currentEdgeIndex != startEdgeIndex);
            }

            COMPG_ASSERT(
                polygonHalfEdges.size() == result.InsertedEdges.size() + 1,
                "Expected the number of monotone sub-polygons to be one more than the number of inserted edges"
            );
            return polygonHalfEdges;
        }
    } // namespace details

    void PolygonTriangulator::Triangulate(DoublyConnectedEdgeList& polygon) const {
        const auto result = Monotonizer.MakeMonotone(polygon);
        const auto halfEdges = details::FindMonotonePolygonsEdges(polygon, result);
        std::ranges::for_each(halfEdges, [this, &polygon](edge_index edgeIndex) {
            Triangulator.Triangulate(polygon, edgeIndex);
        });
    }
} // namespace compg
