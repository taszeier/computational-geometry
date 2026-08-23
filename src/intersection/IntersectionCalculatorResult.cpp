#include "intersection/IntersectionCalculatorResult.hpp"
#include "data_structures/VertexEquivalenceCalculator.hpp"

namespace compg {
    IntersectionCalculatorResult Collapse(const IntersectionCalculatorResult& result, Float epsilon) {
        VertexEquivalenceCalculator equivalence{epsilon};
        auto classes = equivalence.FindEquivalenceClasses(result.Intersections);

        const auto disjointSets = classes.FindDisjointSets();
        IntersectionCalculatorResult collapsedResult;
        collapsedResult.Intersections.reserve(disjointSets.size());
        collapsedResult.IntersectingSegments.reserve(disjointSets.size());

        for (const auto& [rootIndex, equivalentIndices] : disjointSets) {
            collapsedResult.Intersections.push_back(result.Intersections.at(rootIndex));
            collapsedResult.IntersectingSegments.emplace_back();
            std::ranges::for_each(equivalentIndices, [&result, &collapsedResult](const auto i) {
                auto& output = collapsedResult.IntersectingSegments.back();
                std::ranges::copy(result.IntersectingSegments.at(i), std::inserter(output, output.end()));
            });
        }

        return collapsedResult;
    }
} // namespace compg