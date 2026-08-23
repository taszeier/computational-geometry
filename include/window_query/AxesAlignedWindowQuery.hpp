#pragma once

#include "math/primitives/Box.hpp"
#include "window_query/EndPointLocator.hpp"
#include "window_query/IntervalTree.hpp"

namespace compg {
    class AxesAlignedWindowQuery {
    public:
        explicit AxesAlignedWindowQuery(const std::vector<LineSegment2D>& segments);
        std::vector<std::size_t> Query(const Box2D& box) const;

    private:
        EndPointLocator Locator;
        SubsetIntervalTree HorizontalTree;
        SubsetIntervalTree VerticalTree;
    };
} // namespace compg
