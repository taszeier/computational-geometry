#pragma once

#include <ranges>
#include <unordered_map>
#include <vector>

#include "common/Common.hpp"
#include "common/Error.hpp"

namespace compg {
    template <typename ValueType, typename ComparatorType = Less>
    class PriorityQueue {
    public:
        explicit PriorityQueue(const ComparatorType& comparator = {})
            : Comparator(comparator) {}

        /**
         * @brief Initialize the priority queue in linear time.
         * @param range The range of elements to insert.
         * @param comparator An object that orders the elements.
         */
        explicit PriorityQueue(const std::ranges::range auto& range, const ComparatorType& comparator = {})
            : Comparator(comparator) {
            std::ranges::for_each(range, [this](const auto& element) {
                ++Size_;
                ++CountMap[element];
                if (!IndexMap.contains(element)) {
                    IndexMap[element] = Nodes.size();
                    Nodes.push_back(element);
                }
            });
            std::ranges::for_each(
                std::views::iota(0UZ, Nodes.size()) | std::views::reverse, [this](const std::size_t i) { SinkDown(i); }
            );
        }

        /**
         * @return The total number of elements in the queue.
         */
        auto Size() const {
            return Size_;
        }

        /**
         * @param value A value potentially in the queue.
         * @return The number of times the value is in the queue.
         */
        std::size_t Count(const ValueType& value) const {
            return CountMap.contains(value) ? CountMap.at(value) : 0UZ;
        }

        [[nodiscard]] bool IsEmpty() const {
            return Size() == 0;
        }

        ValueType Top() const {
            COMPG_ASSERT(!Nodes.empty(), "The priority queue is empty");
            return Nodes.at(0);
        }

        void Pop() {
            COMPG_ASSERT(!Nodes.empty(), "The priority queue is empty");
            Erase(Top());
        }

        void Insert(const ValueType& value) {
            if (!IndexMap.contains(value)) {
                const auto index = Nodes.size();
                IndexMap[value] = index;
                Nodes.push_back(value);
                SinkUp(index);
            }
            ++CountMap[value];
            ++Size_;
        }

        bool Erase(const ValueType& value) {
            if (!IndexMap.contains(value)) {
                return false;
            }
            if (CountMap.at(value) > 1) {
                --CountMap.at(value);
            } else {
                CountMap.erase(value);
                if (Nodes.size() > 1) {
                    const auto index = IndexMap.at(value);
                    const auto lastValue = Nodes.back();
                    Nodes.at(index) = lastValue;
                    IndexMap.at(lastValue) = index;
                    IndexMap.erase(value);
                    Nodes.pop_back();
                    if (index < Nodes.size()) {
                        Sink(index);
                    }
                } else {
                    Nodes.clear();
                    IndexMap.clear();
                }
            }
            --Size_;
            return true;
        }

    private:
        void Sink(std::size_t index) {
            COMPG_ASSERT(index < Nodes.size(), CreateOutOfRangeMessage("index", index, Nodes.size()));

            if (index != 0 && Comparator(Nodes.at(ParentIndex(index)), Nodes.at(index))) {
                SinkUp(index);
            } else if (
                const auto maxChild = FindMaxChildIndex(index);
                maxChild.has_value() && Comparator(Nodes.at(index), Nodes.at(maxChild.value()))
            ) {
                SinkDown(index);
            }
        }

        void SinkUp(std::size_t index) {
            COMPG_ASSERT(index < Nodes.size(), CreateOutOfRangeMessage("index", index, Nodes.size()));

            bool done = false;
            auto currentIndex = index;
            while (currentIndex > 0 && !done) {
                const auto parentIndex = ParentIndex(currentIndex);
                if (Comparator(Nodes.at(parentIndex), Nodes.at(currentIndex))) {
                    Swap(parentIndex, currentIndex);
                    currentIndex = parentIndex;
                } else {
                    done = true;
                }
            }
        }

        void SinkDown(std::size_t index) {
            COMPG_ASSERT(index < Nodes.size(), CreateOutOfRangeMessage("index", index, Nodes.size()));
            auto currentIndex = index;
            bool done = false;
            while (!done) {
                const auto maxIndex = FindMaxChildIndex(currentIndex);
                if (maxIndex.has_value() && Comparator(Nodes.at(currentIndex), Nodes.at(maxIndex.value()))) {
                    Swap(currentIndex, maxIndex.value());
                    currentIndex = maxIndex.value();
                } else {
                    done = true;
                }
            }
        }

        [[nodiscard]] std::optional<std::size_t> FindMaxChildIndex(std::size_t index) const {
            COMPG_ASSERT(index < Nodes.size(), CreateOutOfRangeMessage("index", index, Nodes.size()));
            const auto leftIndex = 2 * index + 1;
            if (leftIndex < Nodes.size()) {
                auto maxIndex = leftIndex;
                const auto rightIndex = leftIndex + 1;
                if (rightIndex < Nodes.size() && Comparator(Nodes.at(leftIndex), Nodes.at(rightIndex))) {
                    maxIndex = rightIndex;
                }
                return maxIndex;
            }
            return std::nullopt;
        }

        void Swap(std::size_t index1, std::size_t index2) {
            std::swap(IndexMap.at(Nodes.at(index1)), IndexMap.at(Nodes.at(index2)));
            std::swap(Nodes.at(index1), Nodes.at(index2));
        }

        [[nodiscard]] std::size_t ParentIndex(std::size_t index) const {
            COMPG_ASSERT(index < Nodes.size(), CreateOutOfRangeMessage("index", index, Nodes.size()));
            COMPG_ASSERT(index != 0, "The root node does not have a parent");
            return (index - 1) / 2;
        }

    private:
        std::vector<ValueType> Nodes;
        std::unordered_map<ValueType, std::size_t> IndexMap;
        std::unordered_map<ValueType, std::size_t> CountMap;
        ComparatorType Comparator;
        std::size_t Size_{};
    };
} // namespace compg
