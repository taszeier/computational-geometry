#pragma once

#include "common/Common.hpp"

#include <memory>
#include <unordered_set>

#include "math/primitives/Interval.hpp"
#include "math/primitives/LineSegment.hpp"
#include "range_query/layered_range_tree/LayeredRangeTree.hpp"

namespace compg {
    struct MiddleSegments {
        LayeredRangeTree<2UZ> LeftEndPoints;
        LayeredRangeTree<2UZ> RightEndPoints;
        std::unordered_map<std::size_t, std::size_t> SegmentIndex;
    };

    struct IntervalTreeNode {
        Float Median;
        MiddleSegments MiddleSegments;
        std::unique_ptr<IntervalTreeNode> LeftChild = nullptr;
        std::unique_ptr<IntervalTreeNode> RightChild = nullptr;

        constexpr bool IsLeaf() const {
            return LeftChild == nullptr && RightChild == nullptr;
        }
    };

    constexpr bool IsHorizontal(const LineSegment2D& segment, Float epsilon = EPSILON) {
        return math::IsZero(segment[0][1] - segment[1][1], epsilon);
    }
    constexpr bool IsVertical(const LineSegment2D& segment, Float epsilon = EPSILON) {
        return math::IsZero(segment[0][0] - segment[1][0], epsilon);
    }

    class IntervalTree {
    public:
        using node_type = IntervalTreeNode;
        using pointer_type = std::unique_ptr<node_type>;

        static IntervalTree CreateHorizontal(const std::vector<LineSegment2D>& segments);
        static IntervalTree CreateVertical(const std::vector<LineSegment2D>& segments);

        struct QueryRegion {
            Interval<Float> Interval;
            Float Location;
        };

        [[nodiscard]] std::vector<std::size_t> Query(const QueryRegion& queryRegion) const;
        void Query(const QueryRegion& queryRegion, std::unordered_set<std::size_t>& output) const;

    private:
        explicit IntervalTree(pointer_type root)
            : Root(std::move(root)) {}

    private:
        pointer_type Root = nullptr;
    };

    struct SubsetIntervalTree {
        IntervalTree Tree;
        std::unordered_map<std::size_t, std::size_t> OriginalIndex;

        void Query(const IntervalTree::QueryRegion& queryRegion, std::unordered_set<std::size_t>& output) const {
            const auto queryResult = Tree.Query(queryRegion);
            std::ranges::copy(
                queryResult | std::views::transform([this](const auto i) { return OriginalIndex.at(i); }),
                std::inserter(output, output.end())
            );
        }
    };
} // namespace compg
