#pragma once

#include "window_query/EndPointLocator.hpp"
#include "window_query/SegmentTree.hpp"

namespace compg {
    class WindowQuery {
    public:
        explicit WindowQuery(const std::vector<LineSegment2D>& segments);
        std::vector<std::size_t> Query(const Box2D& box) const;

    private:
        EndPointLocator Locator;
        HorizontalSegmentTree HorizontalTree;
        VerticalSegmentTree VerticalTree;
    };
} // namespace compg
