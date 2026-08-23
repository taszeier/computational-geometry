#include "window_query/EndPointLocator.hpp"

namespace compg {
    EndPointLocator EndPointLocator::Create(const std::vector<LineSegment2D>& segments) {
        std::vector<Vertex2D> endPoints;
        endPoints.reserve(segments.size() * 2);
        map_type segmentIndex;
        std::ranges::for_each(segments | std::views::enumerate, [&segmentIndex, &endPoints](const auto& tup) {
            const auto& [index, segment] = tup;
            segmentIndex[endPoints.size()] = index;
            endPoints.push_back(segment[0]);
            segmentIndex[endPoints.size()] = index;
            endPoints.push_back(segment[1]);
        });

        return EndPointLocator{tree_type{endPoints}, segmentIndex};
    }

    void EndPointLocator::Query(const Box2D& box, std::unordered_set<std::size_t>& segmentIndices) const {
        const query_type query;
        const auto endPointRecords = query.Query(EndPointTree, {box});
        std::ranges::for_each(endPointRecords, [this, &segmentIndices](const VertexRecord<2UZ>& record) {
            segmentIndices.insert(SegmentIndex.at(record.Index));
        });
    }
} // namespace compg