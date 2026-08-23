#pragma once

#include "common/Common.hpp"
#include "common/Error.hpp"
#include <ranges>

namespace compg {

    template <std::ranges::range RangeType, typename ComparatorType = Less, typename ProjectorType = Identity>
    constexpr bool IsSorted(RangeType&& range, ComparatorType comparator = {}, ProjectorType projector = {}) {
        return std::ranges::none_of(
            range | std::views::transform(projector) | std::views::slide(2), [comparator](const auto& pair) {
                const auto it1 = pair.begin();
                const auto it2 = std::next(it1);
                return comparator(*it2, *it1);
            }
        );
    }

    template <std::ranges::range RangeType, typename ComparatorType = Less, typename ProjectorType = Identity>
    constexpr bool IsStrictlySorted(RangeType&& range, ComparatorType comparator = {}, ProjectorType projector = {}) {
        return std::ranges::all_of(
            range | std::views::transform(projector) | std::views::slide(2), [comparator](const auto& pair) {
                const auto it1 = pair.begin();
                const auto it2 = std::next(it1);
                return comparator(*it1, *it2);
            }
        );
    }

    template <typename ValueType, typename ComparatorType = Less, typename ProjectorType = Identity>
    std::vector<std::size_t> BisectLeftMany(
        const std::vector<ValueType>& values, const std::vector<ValueType>& queries, ComparatorType comparator = {},
        ProjectorType projector = {}
    ) {
        COMPG_ASSERT(IsSorted(values, comparator, projector), "parameter `values` is not sorted");
        COMPG_ASSERT(IsSorted(queries, comparator, projector), "parameter `queries` is not sorted");

        std::vector<std::size_t> result(queries.size(), values.size());
        std::size_t i = 0;
        std::size_t j = 0;

        while (i < queries.size() && j < values.size()) {
            const auto projection = projector(queries[i]);
            while (j < values.size() && comparator(projector(values[j]), projection)) {
                ++j;
            }
            if (j != values.size()) {
                result[i] = j;
            }
            ++i;
        }
        return result;
    }

    template <
        std::ranges::range RangeType, typename QueryType, typename ComparatorType = Less,
        typename ProjectorType = Identity>
    std::size_t BisectLeft(
        RangeType&& range, const QueryType& query, ComparatorType comparator = {}, ProjectorType projector = {}
    ) {
        const auto it = std::ranges::lower_bound(range, query, comparator, projector);
        return std::distance(range.begin(), it);
    }

    /**
     * @brief Partition the elements so that the median is in its proper sorted
     * position, with all elements before it not greater than the median, and
     * all elements after it not less than the median. The median of an even
     * number of elements is considered to be the element at position n / 2 -
     * 1, where n is the number of elements.
     * @tparam Range The type of the range.
     * @tparam ComparatorType The type of the comparator.
     * @param range A range of elements.
     * @param comparator A comparator.
     * @return An iterator to the median element.
     */
    template <std::ranges::sized_range Range, typename ComparatorType = Less, typename ProjectorType = Identity>
    constexpr auto PartitionMedian(Range&& range, ComparatorType&& comparator = {}, ProjectorType&& projector = {}) {
        COMPG_ASSERT(range.size() > 0, "Range cannot be empty");
        const auto medianIdx = range.size() / 2 - ((range.size() % 2 == 1) ? 0 : 1);
        const auto medianIterator = std::next(range.begin(), medianIdx);
        std::ranges::nth_element(
            range, medianIterator, std::forward<ComparatorType>(comparator), std::forward<ProjectorType>(projector)
        );

        return medianIterator;
    }

    /**
     * @brief Returns the indices that would sort a range.
     */
    template <typename ComparatorType = Less, typename ProjectorType = Identity>
    auto ArgSort(
        const std::ranges::random_access_range auto& range, ComparatorType comparator = {}, ProjectorType projector = {}
    ) {
        std::vector<std::size_t> result(range.size());
        std::ranges::iota(result, 0UZ);
        std::ranges::sort(result, comparator, [&range, projector](std::size_t i) { return projector(range[i]); });

        return result;
    }

    /**
     * @brief Return the largest element that satisfies a predicate or std::nullopt if the predicate is false for all
     * elements.
     */
    template <
        typename Iterator, typename PredicateType, typename ComparatorType = Less, typename ProjectorType = Identity>
    auto MaximumWithProperty(
        Iterator begin, Iterator end, PredicateType&& predicate, ComparatorType comparator, ProjectorType projector = {}
    ) {
        std::optional<Iterator> result{};

        for (auto it = begin; it != end; ++it) {
            if (predicate(*it)) {
                if (!result.has_value() || comparator(projector(*result.value()), projector(*it))) {
                    result = it;
                }
            }
        }
        return result;
    }

    /**
     * @brief Return the largest element that satisfies a predicate or std::nullopt if the predicate is false for all
     * elements.
     */
    template <typename PredicateType, typename ComparatorType = Less, typename ProjectorType = Identity>
    auto MaximumWithProperty(
        const std::ranges::range auto& range, PredicateType&& predicate, ComparatorType comparator,
        ProjectorType projector = {}
    ) {
        return MaximumWithProperty(
            range.begin(), range.end(), std::forward<PredicateType>(predicate), comparator, projector
        );
    }

} // namespace compg