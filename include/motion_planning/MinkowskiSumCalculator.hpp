#pragma once
#include "math/Geometry.hpp"
#include "math/primitives/Polygon.hpp"

namespace compg {
    class MinkowskiSumCalculator {
    public:
        /**
         * @brief Find the Minkowski sum of two polygons.
         * @param p1 The first polygon.
         * @param p2 The second polygon.
         * @return Their Minkowski sum.
         */
        [[nodiscard]] Polygon FindMinkowskiSum(const Polygon& p1, const Polygon& p2) const;

        /**
         * @brief Find the Minkowski sum of two convex polygons.
         * @param p1 The first convex polygon.
         * @param p2 The second convex polygon.
         * @return Their Minkowski sum.
         */
        [[nodiscard]] Polygon FindConvexSum(const Polygon& p1, const Polygon& p2) const;

        /**
         * @brief Find the Minkowski sum of a polygon and a convex polygon.
         * @param polygon A polygon.
         * @param convexPolygon A convex polygon.
         * @return Their Minkowski sum.
         */
        [[nodiscard]] Polygon FindHalfConvexSum(const Polygon& polygon, const Polygon& convexPolygon) const;
    };
} // namespace compg
