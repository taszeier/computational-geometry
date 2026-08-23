#include "window_query/IntervalTree.hpp"

#include <unordered_set>

#include "range_query/layered_range_tree/LayeredRangeTreeQuery.hpp"

namespace compg {
    namespace details {
        using node_type = IntervalTree::node_type;

        struct Segment {
            Float LeftEndPoint;
            Float RightEndPoint;
            Float Height;
        };

        struct SegmentRecord {
            Segment Value;
            std::size_t Index;
        };

        std::vector<Float> GetEndPoints(std::ranges::range auto&& segmentRecords) {
            std::vector<Float> endPoints;
            endPoints.reserve(segmentRecords.size() * 2);
            std::ranges::for_each(segmentRecords, [&endPoints](const SegmentRecord& record) {
                endPoints.push_back(record.Value.LeftEndPoint);
                endPoints.push_back(record.Value.RightEndPoint);
            });
            return endPoints;
        }

        MiddleSegments CreateMiddleSegments(const std::vector<SegmentRecord>& segmentRecords) {
            const auto leftEndPoints = segmentRecords | std::views::transform([](const SegmentRecord& record) {
                                           return Vertex2D{record.Value.LeftEndPoint, record.Value.Height};
                                       })
                                       | std::ranges::to<std::vector>();
            LayeredRangeTree<2UZ> leftTree{leftEndPoints};
            const auto rightEndPoints = segmentRecords | std::views::transform([](const SegmentRecord& record) {
                                            return Vertex2D{record.Value.RightEndPoint, record.Value.Height};
                                        })
                                        | std::ranges::to<std::vector>();
            LayeredRangeTree<2UZ> rightTree{rightEndPoints};

            std::unordered_map<std::size_t, std::size_t> segmentIndex;
            std::ranges::for_each(segmentRecords | std::views::enumerate, [&segmentIndex](const auto& tup) {
                const auto& [index, record] = tup;
                segmentIndex[index] = record.Index;
            });

            return {std::move(leftTree), std::move(rightTree), segmentIndex};
        }

        std::unique_ptr<node_type> CreateIntervalTreeNode(std::ranges::range auto&& segmentRecords) {
            if (segmentRecords.size() == 0) {
                return nullptr;
            }

            auto endPoints = GetEndPoints(segmentRecords);
            const auto median = *PartitionMedian(endPoints);
            std::vector<SegmentRecord> leftRecords;
            std::vector<SegmentRecord> rightRecords;
            std::vector<SegmentRecord> middleRecords;
            std::ranges::for_each(segmentRecords, [&](const SegmentRecord& record) {
                if (record.Value.RightEndPoint < median) {
                    leftRecords.push_back(record);
                } else if (median < record.Value.LeftEndPoint) {
                    rightRecords.push_back(record);
                } else {
                    middleRecords.push_back(record);
                }
            });

            auto leftChild = CreateIntervalTreeNode(leftRecords);
            auto rightChild = CreateIntervalTreeNode(rightRecords);
            return std::make_unique<node_type>(
                median, CreateMiddleSegments(middleRecords), std::move(leftChild), std::move(rightChild)
            );
        }

        void QueryLeft(
            const IntervalTree::QueryRegion& queryRegion, const MiddleSegments& middleSegments,
            std::unordered_set<std::size_t>& output
        ) {
            const LayeredRangeTreeQuery<2UZ> treeQuery;
            LayeredRangeTreeQueryRegion<2UZ> vertexQueryRegion{
                {{-std::numeric_limits<Float>::infinity(), queryRegion.Interval.Lower},
                 {queryRegion.Location, queryRegion.Interval.Upper}}
            };
            const auto queryResult = treeQuery.Query(middleSegments.LeftEndPoints, vertexQueryRegion);

            std::ranges::for_each(queryResult, [&middleSegments, &output](const VertexRecord<2UZ>& record) {
                output.insert(middleSegments.SegmentIndex.at(record.Index));
            });
        }

        void QueryRight(
            const IntervalTree::QueryRegion& queryRegion, const MiddleSegments& middleSegments,
            std::unordered_set<std::size_t>& output
        ) {
            const LayeredRangeTreeQuery<2UZ> treeQuery;
            LayeredRangeTreeQueryRegion<2UZ> vertexQueryRegion{
                {{queryRegion.Location, queryRegion.Interval.Lower},
                 {std::numeric_limits<Float>::infinity(), queryRegion.Interval.Upper}}
            };
            const auto queryResult = treeQuery.Query(middleSegments.RightEndPoints, vertexQueryRegion);

            std::ranges::for_each(queryResult, [&middleSegments, &output](const VertexRecord<2UZ>& record) {
                output.insert(middleSegments.SegmentIndex.at(record.Index));
            });
        }

    } // namespace details

    IntervalTree IntervalTree::CreateHorizontal(const std::vector<LineSegment2D>& segments) {
        std::ranges::for_each(segments, [](const auto& segment) {
            COMPG_ASSERT(IsHorizontal(segment), "Expected all segments to be horizontal");
        });

        auto segmentRecords = segments | std::views::enumerate | std::views::transform([](const auto& tup) {
                                  const auto& [index, segment] = tup;
                                  const Float left = std::min(segment[0][0], segment[1][0]);
                                  const Float right = std::max(segment[0][0], segment[1][0]);
                                  const Float height = segment[0][1];

                                  return details::SegmentRecord{{left, right, height}, static_cast<std::size_t>(index)};
                              })
                              | std::ranges::to<std::vector>();
        return IntervalTree{details::CreateIntervalTreeNode(segmentRecords)};
    }

    IntervalTree IntervalTree::CreateVertical(const std::vector<LineSegment2D>& segments) {
        std::ranges::for_each(segments, [](const auto& segment) {
            COMPG_ASSERT(IsVertical(segment), "Expected all segments to be vertical");
        });

        auto segmentRecords = segments | std::views::enumerate | std::views::transform([](const auto& tup) {
                                  const auto& [index, segment] = tup;
                                  const Float left = std::min(segment[0][1], segment[1][1]);
                                  const Float right = std::max(segment[0][1], segment[1][1]);
                                  const Float height = segment[0][0];

                                  return details::SegmentRecord{{left, right, height}, static_cast<std::size_t>(index)};
                              })
                              | std::ranges::to<std::vector>();
        return IntervalTree{details::CreateIntervalTreeNode(segmentRecords)};
    }

    void IntervalTree::Query(const QueryRegion& queryRegion, std::unordered_set<std::size_t>& output) const {
        auto nodePtr = Root.get();
        while (nodePtr != nullptr) {
            const auto& [Median, MiddleSegments, LeftChild, RightChild] = *nodePtr;
            if (queryRegion.Location < Median) {
                details::QueryLeft(queryRegion, MiddleSegments, output);
                nodePtr = LeftChild.get();
            } else {
                details::QueryRight(queryRegion, MiddleSegments, output);
                nodePtr = RightChild.get();
            }
        }
    }

    std::vector<std::size_t> IntervalTree::Query(const QueryRegion& queryRegion) const {
        std::unordered_set<std::size_t> result;
        Query(queryRegion, result);
        return result | std::ranges::to<std::vector>();
    }

} // namespace compg
