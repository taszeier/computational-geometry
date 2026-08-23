#pragma once

#include "common/Common.hpp"
#include "math/primitives/Interval.hpp"
#include "window_query/SegmentQueryRegion.hpp"

#include <memory>
#include <set>

#include "data_structures/AvlTree.hpp"
#include "math/Geometry.hpp"
#include "math/primitives/Line.hpp"
#include "math/primitives/LineSegment.hpp"

namespace compg {
    struct SegmentRecord {
        std::size_t Index;
        LineSegment2D Segment;
    };

    struct HorizontalProjector {
        static constexpr auto operator()(const auto& x) {
            return Project(x);
        }

        [[nodiscard]] static constexpr Float Project(const Vertex2D& vertex) {
            return vertex[0];
        }

        [[nodiscard]] static constexpr Float ProjectOrthogonal(const Vertex2D& vertex) {
            return vertex[1];
        }

        [[nodiscard]] static auto ProjectBack(const SegmentQueryRegion& region) {
            return std::tuple{
                Vertex2D{region.Location, region.Interval.Lower}, Vertex2D{region.Location, region.Interval.Upper}
            };
        }
    };

    struct VerticalProjector {
        static constexpr auto operator()(const auto& x) {
            return Project(x);
        }

        [[nodiscard]] static constexpr Float Project(const Vertex2D& vertex) {
            return vertex[1];
        }

        [[nodiscard]] static constexpr Float ProjectOrthogonal(const Vertex2D& vertex) {
            return vertex[0];
        }

        [[nodiscard]] static auto ProjectBack(const SegmentQueryRegion& region) {
            return std::tuple{
                Vertex2D{region.Interval.Upper, region.Location}, Vertex2D{region.Interval.Lower, region.Location}
            };
        }
    };

    template <typename ProjectorType>
    struct SegmentBelowComparator {
        explicit SegmentBelowComparator(const OpenInterval& interval, const Line2D& bisector)
            : Interval{interval}
            , Bisector{bisector} {}

        bool operator()(const SegmentRecord& record1, const SegmentRecord& record2) const noexcept {
            const auto intersection1 = FindIntersection(Bisector, record1.Segment).value_or(record1.Segment[0]);
            const auto intersection2 = FindIntersection(Bisector, record2.Segment).value_or(record2.Segment[0]);

            return Projector.ProjectOrthogonal(intersection1) < Projector.ProjectOrthogonal(intersection2);
        }

        constexpr bool operator()(const Vertex2D& vertex, const SegmentRecord& record) const {
            COMPG_ASSERT(Interval.Contains(Projector(vertex)), "Expected a vertex inside the slab");
            if (Interval.IsDegenerate()) {
                return Projector.ProjectOrthogonal(vertex) < std::min(
                           Projector.ProjectOrthogonal(record.Segment[0]),
                           Projector.ProjectOrthogonal(record.Segment[1])
                       );
            }
            const auto [v0, v1] = Projector(record.Segment[0]) < Projector(record.Segment[1])
                                      ? std::tuple{record.Segment[0], record.Segment[1]}
                                      : std::tuple{record.Segment[1], record.Segment[0]};

            return FindSide(v0, v1, vertex) == PointSide::Positive;
        }

        constexpr bool operator()(const SegmentRecord& record, const Vertex2D& vertex) const {
            COMPG_ASSERT(Interval.Contains(Projector(vertex)), "Expected a vertex inside the slab");
            if (Interval.IsDegenerate()) {
                return std::max(
                           Projector.ProjectOrthogonal(record.Segment[0]),
                           Projector.ProjectOrthogonal(record.Segment[1])
                       )
                       < Projector.ProjectOrthogonal(vertex);
            }
            const auto [v0, v1] = Projector(record.Segment[0]) < Projector(record.Segment[1])
                                      ? std::tuple{record.Segment[0], record.Segment[1]}
                                      : std::tuple{record.Segment[1], record.Segment[0]};

            return FindSide(v0, v1, vertex) == PointSide::Negative;
        }

        OpenInterval Interval;
        Line2D Bisector;
        ProjectorType Projector;
    };

    constexpr Float MidPoint(const OpenInterval& interval) {
        COMPG_ASSERT(!interval.IsEmpty(), "Expected a non-empty interval");
        return interval.Lower.Value * 0.5 + interval.Upper.Value * 0.5;
    }

    struct HorizontalSegmentBelowComparator : SegmentBelowComparator<HorizontalProjector> {
        explicit HorizontalSegmentBelowComparator(const OpenInterval& interval)
            : SegmentBelowComparator(interval, MakeLine2DFromX(MidPoint(interval))) {}

        using SegmentBelowComparator::operator();
    };

    struct VerticalSegmentBelowComparator : SegmentBelowComparator<VerticalProjector> {
        explicit VerticalSegmentBelowComparator(const OpenInterval& interval)
            : SegmentBelowComparator(interval, MakeLine2DFromY(MidPoint(interval))) {}

        using SegmentBelowComparator::operator();
    };

    struct SegmentTreeNode {
        OpenInterval Interval;
        std::unique_ptr<SegmentTreeNode> LeftChild = nullptr;
        std::unique_ptr<SegmentTreeNode> RightChild = nullptr;
        AvlTree<SegmentRecord> CrossingSegments;
    };

    namespace details {
        template <typename ProjectorType>
        std::vector<OpenInterval> CreateElementaryIntervals(const std::vector<LineSegment2D>& segments) {
            std::set<Float> endPoints;
            const ProjectorType projector{};
            std::ranges::for_each(segments, [&endPoints, projector](const LineSegment2D& segment) {
                endPoints.insert(projector(segment[0]));
                endPoints.insert(projector(segment[1]));
            });

            std::vector<OpenInterval> intervals;
            intervals.reserve(2 * endPoints.size() + 1);

            Float previous = -Infinity;
            for (const Float endPoint : endPoints) {
                intervals.emplace_back(previous, endPoint);
                intervals.emplace_back(endPoint, endPoint, false, false);
                previous = endPoint;
            }
            intervals.emplace_back(previous, Infinity);
            return intervals;
        }

        std::unique_ptr<SegmentTreeNode> CreateSegmentTree(auto begin, auto end) {
            COMPG_ASSERT(begin != end, "Expected at least one interval");
            if (std::next(begin) == end) {
                return std::make_unique<SegmentTreeNode>(*begin);
            }

            auto middle = std::next(begin, (std::distance(begin, end) - 1) / 2);
            auto leftChild = CreateSegmentTree(begin, middle + 1);
            auto rightChild = CreateSegmentTree(middle + 1, end);
            const OpenInterval interval = Union(leftChild->Interval, rightChild->Interval);
            return std::make_unique<SegmentTreeNode>(interval, std::move(leftChild), std::move(rightChild));
        }

        template <typename ProjectorType>
        std::unique_ptr<SegmentTreeNode> CreateSegmentTree(const std::vector<LineSegment2D>& segments) {
            const auto elementaryIntervals = CreateElementaryIntervals<ProjectorType>(segments);
            return CreateSegmentTree(elementaryIntervals.begin(), elementaryIntervals.end());
        }

        template <typename ComparatorType>
        void Insert(SegmentTreeNode* nodePtr, const SegmentRecord& record, const OpenInterval& segmentInterval) {
            COMPG_ASSERT(nodePtr != nullptr, "Expected a valid pointer");

            if (Covers(segmentInterval, nodePtr->Interval)) {
                nodePtr->CrossingSegments.Insert(record, ComparatorType{nodePtr->Interval});
            } else {
                if (auto leftPtr = nodePtr->LeftChild.get();
                    leftPtr != nullptr && Intersects(segmentInterval, leftPtr->Interval)) {
                    Insert<ComparatorType>(leftPtr, record, segmentInterval);
                }
                if (auto rightPtr = nodePtr->RightChild.get();
                    rightPtr != nullptr && Intersects(segmentInterval, rightPtr->Interval)) {
                    Insert<ComparatorType>(rightPtr, record, segmentInterval);
                }
            }
        }

        template <typename ProjectorType>
        OpenInterval Convert(const LineSegment2D& segment) {
            const ProjectorType projector{};
            const auto v0 = projector(segment[0]);
            const auto v1 = projector(segment[1]);
            const Float lower = std::min(v0, v1);
            const Float upper = std::max(v0, v1);
            return {lower, upper, false, false};
        }

        template <typename ComparatorType, typename ProjectorType>
        void Query(const SegmentTreeNode& node, const auto& queryRegion, auto& output) {
            const ProjectorType projector{};
            const auto [lower, upper] = projector.ProjectBack(queryRegion);
            const ComparatorType comparator{node.Interval};
            const auto lowerBound = node.CrossingSegments.LowerBound(lower, comparator);
            const auto upperBound = node.CrossingSegments.UpperBound(upper, comparator);

            std::copy(lowerBound, upperBound, std::inserter(output, output.end()));
        }
    } // namespace details

    template <typename ComparatorType, typename ProjectorType>
    class SegmentTree {
    public:
        using node_type = SegmentTreeNode;
        using pointer_type = std::unique_ptr<node_type>;

        explicit SegmentTree(const std::vector<LineSegment2D>& segments)
            : Root{details::CreateSegmentTree<ProjectorType>(segments)} {

            std::ranges::for_each(segments | std::views::enumerate, [this](const auto& tup) {
                const auto& [index, segment] = tup;
                const SegmentRecord segmentRecord{static_cast<std::size_t>(index), segment};
                const OpenInterval segmentInterval = details::Convert<ProjectorType>(segment);
                details::Insert<ComparatorType>(Root.get(), segmentRecord, segmentInterval);
            });
        }

        void Query(const SegmentQueryRegion& queryRegion, auto& output) const {
            if (Root == nullptr) {
                return;
            }

            auto nodePtr = Root.get();
            bool done = false;
            while (!done) {
                details::Query<ComparatorType, ProjectorType>(*nodePtr, queryRegion, output);
                if (nodePtr->LeftChild != nullptr && nodePtr->RightChild != nullptr) {
                    if (nodePtr->LeftChild->Interval.Contains(queryRegion.Location)) {
                        nodePtr = nodePtr->LeftChild.get();
                    } else {
                        nodePtr = nodePtr->RightChild.get();
                    }
                } else {
                    done = true;
                }
            }
        }

        [[nodiscard]] std::vector<SegmentRecord> Query(const SegmentQueryRegion& queryRegion) const {
            std::set<SegmentRecord, decltype([](const auto& r1, const auto& r2) { return r1.Index < r2.Index; })>
                result;
            Query(queryRegion, result);
            return result | std::ranges::to<std::vector>();
        }

    private:
        pointer_type Root = nullptr;
    };

    using HorizontalSegmentTree = SegmentTree<HorizontalSegmentBelowComparator, HorizontalProjector>;
    using VerticalSegmentTree = SegmentTree<VerticalSegmentBelowComparator, VerticalProjector>;
} // namespace compg
