#include "intersection/SweepLineIntersectionCalculator.hpp"
#include "intersection/IntersectionCalculatorState.hpp"

namespace compg {
    namespace details {
        auto CreateIntersectionCalculatorResult(
            const std::unordered_set<Vertex2D>& intersections,
            const std::unordered_map<Vertex2D, std::unordered_set<std::size_t>>& intersectingSegments
        ) {

            IntersectionCalculatorResult result{};
            std::ranges::for_each(intersections, [&result, &intersectingSegments](const Vertex2D& v) {
                result.Intersections.push_back(v);
                auto& segments = intersectingSegments.at(v);
                result.IntersectingSegments.emplace_back(segments.begin(), segments.end());
            });
            return result;
        }
    } // namespace details

    IntersectionCalculatorResult
    SweepLineIntersectionCalculator::FindIntersections(const segments_type& segments) const {
        std::unordered_set<Vertex2D> intersections;
        std::unordered_map<Vertex2D, std::unordered_set<std::size_t>> intersectingSegments;

        auto collectIntersections = [&intersections, &intersectingSegments](
                                        const Vertex2D& event, [[maybe_unused]] auto maybeLeftIt,
                                        [[maybe_unused]] auto maybeRightIt, const auto& indices
                                    ) {
            if (indices.LowerIndices.size() + indices.CenterIndices.size() + indices.UpperIndices.size() > 1) {
                intersections.insert(event);
                auto& segments = intersectingSegments[event];
                std::ranges::copy(indices.LowerIndices, std::inserter(segments, segments.end()));
                std::ranges::copy(indices.CenterIndices, std::inserter(segments, segments.end()));
                std::ranges::copy(indices.UpperIndices, std::inserter(segments, segments.end()));
            }
        };

        IntersectionCalculatorState state{segments, Epsilon};
        state.Run(collectIntersections);
        return details::CreateIntersectionCalculatorResult(intersections, intersectingSegments);
    }

} // namespace compg