#pragma once

#include "common/Common.hpp"
#include "convex_hull/EdgeList3D.hpp"
#include <vector>

namespace compg {
    template <std::size_t K>
    class ConvexHull;

    template <>
    class ConvexHull<2UZ> {
    public:
        std::vector<Vertex2D> Vertices;
    };

    template <>
    class ConvexHull<3UZ> {
    public:
        EdgeList3D EdgeList;
    };

    using ConvexHull2D = ConvexHull<2UZ>;
    using ConvexHull3D = ConvexHull<3UZ>;

    template <std::size_t K>
    class ConvexHullCalculator {
    public:
        using convex_hull_type = ConvexHull<K>;

        virtual convex_hull_type FindConvexHull(const std::vector<Vertex<K>>& vertices) const = 0;
        ConvexHullCalculator() = default;

        ConvexHullCalculator(const ConvexHullCalculator& other) = delete;
        ConvexHullCalculator& operator=(const ConvexHullCalculator& other) = delete;
        ConvexHullCalculator(ConvexHullCalculator&& other) = delete;
        ConvexHullCalculator& operator=(ConvexHullCalculator&& other) = delete;
        virtual ~ConvexHullCalculator() = default;
    };

} // namespace compg
