#pragma once

#include "data_structures/AvlTree.hpp"
#include "sweep_line/SweepLine.hpp"
#include <queue>
#include <ranges>

#include "data_structures/PriorityQueue.hpp"

namespace compg {
    class LineSegmentProjector {
    public:
        using segments_type = std::vector<LineSegment2D>;
        explicit LineSegmentProjector(const segments_type& segments)
            : Segments(segments) {}

        const LineSegment2D& operator()(std::size_t i) const {
            return Segments.at(i);
        }

    private:
        const segments_type& Segments;
    };

    struct EventPointSegmentIndices {
        using indices_type = std::vector<std::size_t>;

        indices_type LowerIndices;
        indices_type CenterIndices;
        indices_type UpperIndices;
    };

    class IntersectionCalculatorState {
    public:
        using segments_type = std::vector<LineSegment2D>;
        using indices_type = std::vector<std::size_t>;
        using container_type = AvlTree<std::size_t>;
        using segment_map_type = std::unordered_map<Vertex2D, std::vector<std::size_t>>;
        using events_queue_type = PriorityQueue<Vertex2D, VertexBelowComparator>;

    public:
        explicit IntersectionCalculatorState(const segments_type& segments, Float epsilon = EPSILON)
            : Segments{segments}
            , Epsilon{epsilon}
            , Projector{Segments} {

            for (const auto& [segmentIdx, segment] : segments | std::views::enumerate) {
                InsertEvent(segment[0]);
                InsertEvent(segment[1]);
                Upper[UpperVertex(segment)].push_back(segmentIdx);
            }
        }

        template <typename CallbackType>
        void Run(CallbackType&& callback) {
            while (!EventQueue.IsEmpty()) {
                const auto event = EventQueue.Top();
                EventQueue.Pop();
                HandleEvent(event, callback);
            }
        }

        /**
         * @brief Finds the right-most segment to the left (left neighbor)
         * and the left-most segment to the right (right neighbor) of the
         * segments that intersect the event point.
         * @param event The current event point.
         * @return A pair of optional iterators to the left and right
         * neighbors. The iterators point to valid segments if they exist
         * and are std::nullopt if the neighbor does not exist.
         */
        auto FindNeighbors(const Vertex2D& event) const {
            // TODO: compare to event not to this degenerate line segment
            const LineSegment2D sweepLineSegment{event, event};
            auto it = State.LowerBound(sweepLineSegment, LineSegmentLeftToRightComparator{event, Epsilon}, Projector);
            if (it != State.begin() && FindSide(Segments.at(*std::prev(it)), event, Epsilon) == PointSide::Collinear) {
                --it;
            }

            auto leftIt = it;
            while (leftIt != State.begin()
                   && FindSide(Segments.at(*std::prev(leftIt)), event, Epsilon) == PointSide::Collinear) {
                --leftIt;
            }
            auto maybeLeftIt = leftIt == State.begin() ? std::nullopt : std::optional{std::prev(leftIt)};

            auto rightIt = it;
            while (rightIt != State.end() && FindSide(Segments.at(*rightIt), event, Epsilon) == PointSide::Collinear) {
                ++rightIt;
            }
            auto maybeRightIt = rightIt == State.end() ? std::nullopt : std::optional{rightIt};

            if (maybeLeftIt.has_value()) {
                COMPG_ASSERT(*maybeLeftIt != State.end(), "Left neighbor is invalid. Should point to a valid segment.");
            }
            if (maybeRightIt.has_value()) {
                COMPG_ASSERT(
                    *maybeRightIt != State.end(), "Right neighbor is invalid. Should point to a valid segment."
                );
            }

            return std::pair(maybeLeftIt, maybeRightIt);
        }

        /**
         * @brief Collect the segments that intersect the event point
         * grouped by whether the segments end at the event point ("lower"
         * segments) or intersect it in the center ("center" segments).
         * @param maybeLeftIt An iterator to the left neighbor if it exists.
         * @param maybeRightIt An iterator to the right neighbor if it
         * exists.
         * @param event The current event point.
         * @return A pair of vectors of iterators to the lower and center
         * segments, respectively.
         */
        auto CollectIntersectingSegments(
            std::optional<container_type::iterator> maybeLeftIt, std::optional<container_type::iterator> maybeRightIt,
            const Vertex2D& event
        ) {

            std::vector<container_type::iterator> lower;
            std::vector<container_type::iterator> center;

            const auto endIt = maybeRightIt.value_or(State.end());
            const auto startIt = maybeLeftIt.transform([](auto it) { return std::next(it); }).value_or(State.begin());
            for (auto it = startIt; it != endIt; ++it) {
                // auto& iterators = AreEqual(LowerVertex(Segments.at(*it)), event, Epsilon) ? lower : center;
                auto& iterators = LowerVertex(Segments.at(*it)) == event ? lower : center;
                iterators.push_back(it);
            };

            return std::pair(lower, center);
        }

        template <typename CallbackType>
        void HandleEvent(const Vertex2D& event, CallbackType&& callback) {
            const auto [maybeLeftIt, maybeRightIt] = FindNeighbors(event);
            auto [lowerIterators, centerIterators] = CollectIntersectingSegments(maybeLeftIt, maybeRightIt, event);

            EventPointSegmentIndices indices{
                lowerIterators | std::views::transform(&container_type::iterator::operator*)
                    | std::ranges::to<std::vector>(),
                centerIterators | std::views::transform(&container_type::iterator::operator*)
                    | std::ranges::to<std::vector>(),
                Upper[event]
            };
            callback(event, maybeLeftIt, maybeLeftIt, indices);

            RemoveSegments(lowerIterators, centerIterators);
            InsertSegments(event, indices.CenterIndices, indices.UpperIndices);
            FindNewEvents(event, maybeLeftIt, maybeRightIt, indices.CenterIndices, indices.UpperIndices);
        }

        void RemoveSegments(const auto& lowerIterators, const auto& centerIterators) {
            std::ranges::for_each(lowerIterators, [this](auto it) { State.Erase(it); });
            std::ranges::for_each(centerIterators, [this](auto it) { State.Erase(it); });
        }

        void
        InsertSegments(const Vertex2D& event, const indices_type& centerIndices, const indices_type& upperIndices) {
            std::ranges::for_each(centerIndices, [this, &event](const auto& segmentIndex) {
                State.Insert(segmentIndex, LineSegmentLeftToRightComparator{event, Epsilon}, Projector);
            });
            std::ranges::for_each(upperIndices, [this, &event](const auto& segmentIndex) {
                State.Insert(segmentIndex, LineSegmentLeftToRightComparator{event, Epsilon}, Projector);
            });
        }

        void FindNewEvents(
            const Vertex2D& event, auto maybeLeftIt, auto maybeRightIt, const indices_type& centerIndices,
            const indices_type& upperIndices
        ) {
            if (upperIndices.empty() && centerIndices.empty()) {
                if (maybeLeftIt.has_value() && maybeRightIt.has_value()) {
                    FindNewEvent(**maybeLeftIt, **maybeRightIt, event);
                }
            } else {
                maybeLeftIt.and_then([this, &event](auto leftIt) {
                    COMPG_ASSERT(
                        std::next(leftIt) != State.end(),
                        "The left neighbor does not have any segments to the right. This should be impossible."
                    );
                    FindNewEvent(*leftIt, *std::next(leftIt), event);
                    return std::optional{leftIt};
                });

                maybeRightIt.and_then([this, &event](auto rightIt) {
                    COMPG_ASSERT(
                        rightIt != State.begin(),
                        "The right neighbor does not have any segments to the left. This should be impossible."
                    );
                    FindNewEvent(*std::prev(rightIt), *rightIt, event);
                    return std::optional{rightIt};
                });
            }
        }

        void FindNewEvent(std::size_t leftIndex, std::size_t rightIndex, const Vertex2D& event) {
            const auto left = Segments.at(leftIndex);
            const auto right = Segments.at(rightIndex);
            FindIntersection(left, right)
                .and_then([&left, &right, this](const auto& v) -> std::optional<Vertex2D> {
                    if (AreEqual(LowerVertex(left), v, Epsilon) || AreEqual(LowerVertex(right), v, Epsilon)) {
                        return std::nullopt;
                    }
                    return v;
                })
                .transform([&left, &right](const auto& v) {
                    if (left[0][1] == left[1][1]) {
                        return Vertex2D{v[0], left[0][1]};
                    }
                    if (right[0][1] == right[1][1]) {
                        return Vertex2D{v[0], right[0][1]};
                    }
                    return v;
                })
                .and_then([&event](const auto& v) -> std::optional<Vertex2D> {
                    // The intersection can be above the event if the line segment is horizontal due to numerical error.
                    // Add epsilon to mitigate this, but it could probably be done better.
                    // The test below is probably not necessary either...
                    /*if (AreEqual(event, v, Epsilon)) {
                        return std::nullopt;
                    }
                    return VertexBelowComparator::Compare(v, event + Vertex2D{0, Epsilon}) ? std::optional{v}
                                                                                           : std::nullopt;*/
                    return VertexBelowComparator::Compare(v, event) ? std::optional{v} : std::nullopt;
                })
                .and_then([this](const auto& v) {
                    InsertEvent(v);
                    return std::optional{v};
                });
        }

        void InsertEvent(const auto& event) {
            if (EventQueue.Count(event) == 0) {
                EventQueue.Insert(event);
            }
        }

    private:
        const segments_type& Segments;
        segment_map_type Upper;
        events_queue_type EventQueue;
        container_type State;
        Float Epsilon;

        LineSegmentProjector Projector;
    };
} // namespace compg