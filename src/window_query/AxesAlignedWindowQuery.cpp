#include "window_query/AxesAlignedWindowQuery.hpp"
#include <unordered_set>

namespace compg {
    namespace details {
        SubsetIntervalTree CreateHorizontalTree(const std::vector<LineSegment2D>& segments) {
            std::unordered_map<std::size_t, std::size_t> originalIndex;
            std::vector<LineSegment2D> horizontalSegments;
            std::ranges::for_each(
                segments | std::views::enumerate, [&originalIndex, &horizontalSegments](const auto& tup) {
                    const auto& [index, segment] = tup;
                    if (IsHorizontal(segment)) {
                        originalIndex[horizontalSegments.size()] = index;
                        horizontalSegments.push_back(segment);
                    }
                }
            );

            return {IntervalTree::CreateHorizontal(horizontalSegments), originalIndex};
        }

        SubsetIntervalTree CreateVerticalTree(const std::vector<LineSegment2D>& segments) {
            std::unordered_map<std::size_t, std::size_t> originalIndex;
            std::vector<LineSegment2D> verticalSegments;
            std::ranges::for_each(
                segments | std::views::enumerate, [&originalIndex, &verticalSegments](const auto& tup) {
                    const auto& [index, segment] = tup;
                    if (IsVertical(segment)) {
                        originalIndex[verticalSegments.size()] = index;
                        verticalSegments.push_back(segment);
                    }
                }
            );

            return {IntervalTree::CreateVertical(verticalSegments), originalIndex};
        }
    } // namespace details

    AxesAlignedWindowQuery::AxesAlignedWindowQuery(const std::vector<LineSegment2D>& segments)
        : Locator{EndPointLocator::Create(segments)}
        , HorizontalTree{details::CreateHorizontalTree(segments)}
        , VerticalTree{details::CreateVerticalTree(segments)} {

        std::ranges::for_each(segments, [](const auto& segment) {
            COMPG_ASSERT(
                IsHorizontal(segment) || IsVertical(segment),
                "Expected all segments to be either horizontal or vertical"
            );
        });
    }

    std::vector<std::size_t> AxesAlignedWindowQuery::Query(const Box2D& box) const {
        std::unordered_set<std::size_t> segmentIndices;

        Locator.Query(box, segmentIndices);
        const IntervalTree::QueryRegion leftSide{
            Interval<Float>{box.GetLowerCorner()[1], box.GetUpperCorner()[1]}, box.GetLowerCorner()[0]
        };
        HorizontalTree.Query(leftSide, segmentIndices);
        const IntervalTree::QueryRegion topSide{
            Interval<Float>{box.GetLowerCorner()[0], box.GetUpperCorner()[0]}, box.GetUpperCorner()[1]
        };
        VerticalTree.Query(topSide, segmentIndices);

        return segmentIndices | std::ranges::to<std::vector>();
    }
} // namespace compg
