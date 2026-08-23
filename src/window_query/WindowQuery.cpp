#include "window_query/WindowQuery.hpp"

namespace compg {
    WindowQuery::WindowQuery(const std::vector<LineSegment2D>& segments)
        : Locator{EndPointLocator::Create(segments)}
        , HorizontalTree{segments}
        , VerticalTree{segments} {}

    std::vector<std::size_t> WindowQuery::Query(const Box2D& box) const {
        std::unordered_set<std::size_t> segmentIndices;
        Locator.Query(box, segmentIndices);

        const SegmentQueryRegion leftSide{{box.GetLowerCorner()[1], box.GetUpperCorner()[1]}, box.GetLowerCorner()[0]};
        const auto horizontalQueryResult = HorizontalTree.Query(leftSide);
        std::ranges::copy(
            horizontalQueryResult | std::views::transform([](const auto& x) { return x.Index; }),
            std::inserter(segmentIndices, segmentIndices.end())
        );

        const SegmentQueryRegion topSide{{box.GetLowerCorner()[0], box.GetUpperCorner()[0]}, box.GetUpperCorner()[1]};
        const auto verticalQueryResult = VerticalTree.Query(topSide);
        std::ranges::copy(
            verticalQueryResult | std::views::transform([](const auto& x) { return x.Index; }),
            std::inserter(segmentIndices, segmentIndices.end())
        );

        return segmentIndices | std::ranges::to<std::vector>();
    }
} // namespace compg