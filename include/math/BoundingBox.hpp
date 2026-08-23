#pragma once

#include "math/primitives/Box.hpp"

namespace compg {
    template <std::size_t K>
    Box<K> FindBoundingBox(const Box<K>& box1, const Box<K>& box2) {
        const auto lower = ElementWiseMin<K>(box1.GetLowerCorner(), box2.GetLowerCorner());
        const auto upper = ElementWiseMax<K>(box1.GetUpperCorner(), box2.GetUpperCorner());
        return Box<K>{lower, upper};
    }

    template <std::ranges::range VertexRange>
        requires(std::is_same_v<
                 std::ranges::range_value_t<VertexRange>,
                 Vertex<std::ranges::range_value_t<VertexRange>::RowsAtCompileTime>>)
    auto FindBoundingBox(const VertexRange& vertices) {
        COMPG_ASSERT(vertices.size() > 0, "Cannot create a bounding box from an empty range");

        constexpr auto N = std::ranges::range_value_t<VertexRange>::RowsAtCompileTime;
        Vertex<N> lower{};
        Vertex<N> upper{};
        for (std::size_t i : ConvertToArray(std::make_index_sequence<N>())) {
            const auto ithIndexComparator = [i](const auto& v1, const auto& v2) { return v1(i) < v2(i); };

            const auto& minElement = *std::ranges::min_element(vertices, ithIndexComparator);
            lower(i) = minElement(i);
            const auto& maxElement = *std::ranges::max_element(vertices, ithIndexComparator);
            upper(i) = maxElement(i);
        }

        return Box<N>(lower, upper);
    }

    auto FindBoundingBox(const std::ranges::range auto& objects) {
        COMPG_ASSERT(objects.begin() != objects.end(), "Expected at least one object");

        auto resultBox{FindBoundingBox(*objects.begin())};
        std::ranges::for_each(objects | std::views::drop(1), [&resultBox](const auto& object) {
            const auto boundingBox = FindBoundingBox(object);
            resultBox = FindBoundingBox(resultBox, boundingBox);
        });

        return resultBox;
    }

    Box3D FindBoundingBox(const Triangle3D& triangle);
    Box2D FindBoundingBox(const LineSegment2D& segment);
} // namespace compg