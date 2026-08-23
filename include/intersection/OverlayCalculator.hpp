#pragma once

#include "data_structures/DoublyConnectedEdgeList.hpp"
#include "math/Geometry.hpp"

namespace compg {
    namespace details {
        class OverlayCalculatorState {
        public:
            std::unordered_map<std::size_t, DoublyConnectedEdgeList::edge_index> Map;
            std::unordered_set<DoublyConnectedEdgeList::edge_index> LeftEdges;
            std::unordered_set<DoublyConnectedEdgeList::edge_index> RightEdges;
        };
    } // namespace details

    class OverlayCalculator {
    private:
        static void
        FixEdgeOrder(std::vector<DoublyConnectedEdgeList::edge_index>& edgeIndices, DoublyConnectedEdgeList& overlay) {
            std::ranges::sort(edgeIndices, Less{}, [&overlay](auto edgeIndex) {
                return Angle360(overlay.GetEdgeAsVector(edgeIndex), CreateCanonicalVector<2>(0));
            });
            std::ranges::for_each(edgeIndices | std::views::enumerate, [&overlay, &edgeIndices](const auto pair) {
                const auto [i, edgeIndex] = pair;
                const auto twinIndex = overlay.GetTwinIndex(edgeIndex);
                const auto nextIndex = edgeIndices.at((i + 1) % edgeIndices.size());

                overlay.Edges.at(twinIndex).NextIndex = nextIndex;
                overlay.Edges.at(nextIndex).PreviousIndex = twinIndex;
            });
        }

        static void HandlePointPointIntersection(
            const Vertex2D& intersection, const auto& lowerIndices, const auto& upperIndices,
            const details::OverlayCalculatorState& state, DoublyConnectedEdgeList& overlay
        ) {

            std::vector<DoublyConnectedEdgeList::edge_index> edgeIndices;
            edgeIndices.reserve(lowerIndices.size() + upperIndices.size());
            const auto intersectionIndex = overlay.VertexIndexMap.at(intersection);

            auto addIndex = [&edgeIndices, &state, &overlay, intersectionIndex](auto segmentIndex) {
                auto overlayEdgeIndex = state.Map.at(segmentIndex);
                if (overlay.GetOriginIndex(overlayEdgeIndex) != intersectionIndex) {
                    overlayEdgeIndex = overlay.GetTwinIndex(overlayEdgeIndex);
                }
                COMPG_ASSERT(
                    overlay.GetOriginIndex(overlayEdgeIndex) == intersectionIndex,
                    "The edge is not incident to the intersection"
                );

                edgeIndices.push_back(overlayEdgeIndex);
            };
            std::ranges::for_each(lowerIndices, addIndex);
            std::ranges::for_each(upperIndices, addIndex);

            FixEdgeOrder(edgeIndices, overlay);
        }

        static void HandlePointLineIntersection(
            const Vertex2D& intersection, const auto& lowerIndices, std::size_t centerIndex, const auto& upperIndices,
            const details::OverlayCalculatorState& state, DoublyConnectedEdgeList& overlay
        ) {

            const auto edgeIndex = state.Map.at(centerIndex);
            const auto intersectionIndex = overlay.Split(edgeIndex, intersection);

            std::vector<DoublyConnectedEdgeList::edge_index> edgeIndices;
            edgeIndices.reserve(lowerIndices.size() + upperIndices.size() + 2);
            edgeIndices.push_back(overlay.GetNextIndex(edgeIndex));
            edgeIndices.push_back(overlay.GetTwinIndex(edgeIndex));

            auto addEdge = [&edgeIndices, &state, &overlay, intersectionIndex](auto segmentIndex) {
                auto edgeIndex = state.Map.at(segmentIndex);
                if (overlay.GetOriginIndex(edgeIndex) != intersectionIndex) {
                    edgeIndex = overlay.GetTwinIndex(edgeIndex);
                }
                COMPG_ASSERT(
                    overlay.GetOriginIndex(edgeIndex) == intersectionIndex,
                    "The edge is not incident to the intersection"
                );
                edgeIndices.push_back(edgeIndex);
            };
            std::ranges::for_each(lowerIndices, addEdge);
            std::ranges::for_each(upperIndices, addEdge);

            FixEdgeOrder(edgeIndices, overlay);
        }

        static void HandleLineLineIntersection(
            const Vertex2D& intersection, std::size_t segmentIndex1, std::size_t segmentIndex2,
            const details::OverlayCalculatorState& state, DoublyConnectedEdgeList& overlay
        ) {

            const auto edgeIndex1 = state.Map.at(segmentIndex1);
            overlay.Split(edgeIndex1, intersection);
            const auto edgeIndex2 = state.Map.at(segmentIndex2);
            overlay.Split(edgeIndex2, intersection);

            std::vector<DoublyConnectedEdgeList::edge_index> edgeIndices{
                overlay.GetNextIndex(edgeIndex1), overlay.GetTwinIndex(edgeIndex1), overlay.GetNextIndex(edgeIndex2),
                overlay.GetTwinIndex(edgeIndex2)
            };
            FixEdgeOrder(edgeIndices, overlay);
        }

    public:
        /**
         * @brief Overlay two doubly connected edge lists.
         * @param lhs A doubly connected edge list.
         * @param rhs A doubly connected edge list.
         * @return The subdivision of the overlay.
         */
        [[nodiscard]] DoublyConnectedEdgeList
        FindOverlay(const DoublyConnectedEdgeList& lhs, const DoublyConnectedEdgeList& rhs) const;

    private:
        Float Epsilon = EPSILON;
    };
} // namespace compg