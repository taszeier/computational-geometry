#pragma once
#include "math/primitives/LineSegment.hpp"
#include "range_query/layered_range_tree/LayeredRangeTree.hpp"
#include "range_query/layered_range_tree/LayeredRangeTreeQuery.hpp"

#include <unordered_set>

namespace compg {
    struct EndPointLocator {
        using tree_type = LayeredRangeTree<2UZ>;
        using query_type = LayeredRangeTreeQuery<2UZ>;
        using map_type = std::unordered_map<std::size_t, std::size_t>;

        static EndPointLocator Create(const std::vector<LineSegment2D>& segments);
        void Query(const Box2D& box, std::unordered_set<std::size_t>& segmentIndices) const;

        tree_type EndPointTree;
        map_type SegmentIndex;
    };
} // namespace compg
