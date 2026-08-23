#include "point_location/PointLocator.hpp"
#include "common/Random.hpp"
#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include "math/BoundingBox.hpp"

namespace compg {

    namespace details {
        auto CollectUpperEdges(const DoublyConnectedEdgeList& edgeList) {
            std::vector<LineSegment2D> segments;

            WalkUndirectedEdges(edgeList, [&edgeList, &segments](auto edgeIndex) {
                const auto origin = edgeList.GetVertex(edgeList.GetOriginIndex(edgeIndex)).Vertex;
                const auto destination = edgeList.GetVertex(edgeList.GetDestinationIndex(edgeIndex)).Vertex;

                const auto [v0, v1] = LexicographicalLess<Vertex2D>::Compare(origin, destination)
                                          ? std::tuple{destination, origin}
                                          : std::tuple{origin, destination};

                segments.emplace_back(v0, v1);
            });

            return segments;
        }

        auto FindEdgeMap(const DoublyConnectedEdgeList& edgeList) {
            std::unordered_map<LineSegment2D, DoublyConnectedEdgeList::edge_index> edgeMap;

            WalkUndirectedEdges(edgeList, [&edgeList, &edgeMap](auto edgeIndex) {
                const auto origin = edgeList.GetVertex(edgeList.GetOriginIndex(edgeIndex)).Vertex;
                const auto destination = edgeList.GetVertex(edgeList.GetDestinationIndex(edgeIndex)).Vertex;

                if (LexicographicalLess<Vertex2D>::Compare(origin, destination)) {
                    const auto [v0, v1] = std::tuple{destination, origin};
                    const LineSegment2D segment{v0, v1};
                    edgeMap[segment] = edgeList.GetTwinIndex(edgeIndex);
                } else {
                    const auto [v0, v1] = std::tuple{origin, destination};
                    const LineSegment2D segment{v0, v1};
                    edgeMap[segment] = edgeIndex;
                }
            });

            return edgeMap;
        }
    } // namespace details

    PointLocator::PointLocator(const DoublyConnectedEdgeList& edgeList, std::size_t seed)
        : PointLocator(edgeList, details::CollectUpperEdges(edgeList), seed) {}

    PointLocator::PointLocator(
        const DoublyConnectedEdgeList& edgeList, const std::vector<LineSegment2D>& segments, std::size_t seed
    )
        : Map{Pad(FindBoundingBox(segments), 1.0, 1.0)}
        , SearchStructure{0}
        , EdgeList{edgeList} {
        EdgeMap = details::FindEdgeMap(edgeList);
        const auto permutation = RandomIndexPermutation(segments, seed);

        std::ranges::for_each(permutation, [this, &segments](auto i) {
            const auto& segment = segments.at(i);
            const auto intersectingTrapezoids = SearchStructure.FollowSegment(Map, segment);
            const auto splitResult = Map.Split(segment, intersectingTrapezoids);
            SearchStructure.UpdateAfterSplit(intersectingTrapezoids, splitResult, segment);
        });
    }

    using Location = PointLocator::Location;

    Location PointLocator::LocatePoint(const Vertex2D& point) const {
        if (!Map.GetBoundingBox().Contains(point)) {
            return Location{Location::state_type{std::in_place_index<0UZ>, EdgeList.FindUnboundedFaceIndex()}};
        }
        const auto searchResult = SearchStructure.Search(point);
        return std::visit(
            Overloads{
                [this](const TrapezoidalSearchStructure::trapezoid_index trapezoidIndex) {
                    const auto topSegment = Map.GetTrapezoid(trapezoidIndex).TopSegment;
                    const auto faceIndex = EdgeMap.contains(topSegment) ? EdgeList.GetFaceIndex(EdgeMap.at(topSegment))
                                                                        : EdgeList.FindUnboundedFaceIndex();
                    return Location{Location::state_type{std::in_place_index<0UZ>, faceIndex}};
                },
                [this](const LineSegment2D& segment) {
                    return Location{Location::state_type{std::in_place_index<1UZ>, EdgeMap.at(segment)}};
                },
                [this](const Vertex2D& vertex) {
                    return Location{Location::state_type{std::in_place_index<2UZ>, EdgeList.GetVertexIndex(vertex)}};
                }
            },
            searchResult.State
        );
    }
} // namespace compg