#pragma once

#include "common/Vertex.hpp"
#include "data_structures/AvlTree.hpp"
#include "math/Math.hpp"
#include "math/Polynomials.hpp"
#include "voronoi/Common.hpp"

namespace compg {
    /**
     * @brief Create a quadratic polynomial whose graph contains the vertices equidistant from the site and the sweep
     * line.
     * @param site The location of the site.
     * @param sweepLineY The position of the sweep line. Must be below the site.
     * @return The quadratic polynomial.
     */
    inline QuadraticPolynomial CreateSitePolynomial(const Vertex2D& site, Float sweepLineY) {
        COMPG_ASSERT(site[1] > sweepLineY, "Expected the sweep line to be below the site");
        const Float inv = 1 / (site[1] - sweepLineY);
        const Float a = 0.5 * inv;
        const Float b = -site[0] * inv;
        const Float c = (site.squaredNorm() - sweepLineY * sweepLineY) * 0.5 * inv;

        return {{c, b, a}};
    }

    /**
     * @brief Compare the location of break points on the beach line to a vertex on the sweep line.
     */
    struct BreakPointLess {
        explicit BreakPointLess(const std::vector<Vertex2D>& sites)
            : Sites(sites) {}

        /**
         * @param vertex A point on the sweep line.
         * @param breakPoint A break point on the beach line.
         * @return Whether the x-coordinate of the vertex is less than the x-coordinate of the break point.
         */
        constexpr bool operator()(const Vertex2D& vertex, const BreakPoint& breakPoint) const {
            const auto intersection = FindIntersection(breakPoint, vertex);
            return vertex[0] < intersection;
        }

        /**
         * @param vertex A point on the sweep line.
         * @param breakPoint A break point on the beach line.
         * @return Whether the x-coordinate of the break point is less than the x-coordinate of the vertex.
         */
        constexpr bool operator()(const BreakPoint& breakPoint, const Vertex2D& vertex) const {
            const auto intersection = FindIntersection(breakPoint, vertex);
            return intersection < vertex[0];
        }

    private:
        /**
         * @param breakPoint A break point on the beach line.
         * @param vertex A vertex on the sweep line.
         * @return The x-coordinate of the break point, i.e., where the arcs intersect.
         */
        [[nodiscard]] Float FindIntersection(const BreakPoint& breakPoint, const Vertex2D& vertex) const {
            const auto pi = Sites.at(breakPoint.LeftSite.Id);
            const auto pj = Sites.at(breakPoint.RightSite.Id);
            COMPG_ASSERT(
                !(math::IsZero(pi[1] - vertex[1]) && math::IsZero(pj[1] - vertex[1])), "This should not be possible"
            );
            return FindIntersectionBetween(pi, pj, vertex[1]);
        }

        /**
         * @param left The location of the site defining the arc directly left of the break point.
         * @param right The location of the site defining the arc directly right of the break point.
         * @param sweepLineY The location of the sweep line.
         * @return The x-coordinate of the break point, i.e., where the arcs intersect.
         */
        static Float FindIntersectionBetween(const Vertex2D& left, const Vertex2D& right, Float sweepLineY) {
            if (math::IsZero(left[1] - sweepLineY)) {
                return left[0];
            }
            if (math::IsZero(right[1] - sweepLineY)) {
                return right[0];
            }

            const auto leftPoly = CreateSitePolynomial(left, sweepLineY);
            const auto rightPoly = CreateSitePolynomial(right, sweepLineY);
            const QuadraticPolynomialRoots roots = FindRealRoots(leftPoly - rightPoly);
            return GetXCoordinate(roots, left, right);
        }

        static Float
        GetXCoordinate(const QuadraticPolynomialRoots& roots, const Vertex2D& left, const Vertex2D& right) {
            if (math::IsZero(left[1] - right[1])) {
                COMPG_ASSERT(left[0] < right[0], "There is no left-right intersection");
                return roots.MinRoot.value();
            }
            if (left[1] > right[1]) {
                return roots.MinRoot.value();
            }
            return roots.MaxRoot.value();
        }

        const std::vector<Vertex2D>& Sites;
    };

    class BeachLine {
    public:
        using break_points_type = AvlTree<BreakPoint>;

        /**
         * @param sites A non-empty vector of unique sites.
         * @param collinearSites A non-empty vector of sites that all have the largest y-coordinate.
         */
        explicit BeachLine(const std::vector<Vertex2D>& sites, const std::vector<SiteEventId>& collinearSites)
            : Sites(sites)
            , InitialSite(collinearSites.at(0)) {
            COMPG_ASSERT(!collinearSites.empty(), "Expected at least one site to initialize the beach line");

            std::ranges::for_each(collinearSites | std::views::slide(2), [this](const auto& iterable) {
                const auto leftSite = *iterable.begin();
                const auto rightSite = *std::next(iterable.begin());
                BreakPoints.InsertBefore(BreakPoint{.LeftSite = leftSite, .RightSite = rightSite}, BreakPoints.end());
            });
        }

        /**
         * @param vertex A vertex on the sweep line.
         * @return The arc that is above a vertex on the sweep line.
         */
        [[nodiscard]] Arc FindArcAbove(const Vertex2D& vertex) const {
            if (BreakPoints.begin() == BreakPoints.end()) {
                return Arc{.LeftSite = std::nullopt, .MiddleSite = InitialSite, .RightSite = std::nullopt};
            }

            const auto it = BreakPoints.UpperBound(vertex, BreakPointLess{Sites});
            if (it != BreakPoints.end()) {
                std::optional<SiteEventId> leftSite;
                if (it != BreakPoints.begin()) {
                    const auto prev = std::prev(it);
                    leftSite = prev->LeftSite;
                }
                return Arc{.LeftSite = leftSite, .MiddleSite = it->LeftSite, .RightSite = it->RightSite};
            }
            const auto last = std::prev(BreakPoints.end());
            return Arc{.LeftSite = last->LeftSite, .MiddleSite = last->RightSite, .RightSite = std::nullopt};
        }

        /**
         * @param site The next site intersected by the sweep line.
         * @return Iterators to the left and right break points created by the insertion.
         */
        auto InsertArc(SiteEventId site) {
            if (BreakPoints.begin() == BreakPoints.end()) {
                COMPG_ASSERT(
                    !math::IsZero(Sites.at(InitialSite.Id)[1] - Sites.at(site.Id)[1]),
                    "Expected the site to be below the initial site"
                );
                const auto leftIt = BreakPoints.InsertBefore(
                    BreakPoint{.LeftSite = InitialSite, .RightSite = site}, BreakPoints.end()
                );
                const auto rightIt = BreakPoints.InsertBefore(
                    BreakPoint{.LeftSite = site, .RightSite = InitialSite}, BreakPoints.end()
                );
                return std::tuple{leftIt, rightIt};
            }
            const auto it = BreakPoints.UpperBound(Sites.at(site.Id), BreakPointLess{Sites});
            const auto siteAbove = it == BreakPoints.end() ? std::prev(BreakPoints.end())->RightSite : it->LeftSite;
            const auto leftIt = BreakPoints.InsertBefore(BreakPoint{.LeftSite = siteAbove, .RightSite = site}, it);
            const auto rightIt = BreakPoints.InsertBefore(BreakPoint{.LeftSite = site, .RightSite = siteAbove}, it);
            return std::tuple{leftIt, rightIt};
        }

        /**
         * @brief Erase an arc from the beach line when two break points meet.
         * @param leftBreakPoint An iterator to the left break point.
         * @param rightBreakPoint An iterator to the right break point.
         * @return Optional iterators to the break point left of the left break point and right of the right break
         * point.
         */
        auto EraseArc(auto leftBreakPoint, auto rightBreakPoint) {
            COMPG_ASSERT(leftBreakPoint != BreakPoints.end(), "Expected a valid iterator");
            COMPG_ASSERT(rightBreakPoint != BreakPoints.end(), "Expected a valid iterator");
            COMPG_ASSERT(std::next(leftBreakPoint) == rightBreakPoint, "Expected the break points to be adjacent");

            std::optional<break_points_type::iterator> farLeft;
            std::optional<break_points_type::iterator> farRight;
            if (leftBreakPoint != BreakPoints.begin()) {
                const auto farLeftIt = std::prev(leftBreakPoint);
                if (farLeftIt->LeftSite != leftBreakPoint->RightSite) {
                    farLeft = farLeftIt;
                }
            }
            const auto farRightIt = std::next(rightBreakPoint);
            if (farRightIt != BreakPoints.end() && farRightIt->RightSite != leftBreakPoint->RightSite) {
                farRight = farRightIt;
            }
            leftBreakPoint->RightSite = rightBreakPoint->RightSite;
            BreakPoints.Erase(rightBreakPoint);

            return std::tuple{farLeft, leftBreakPoint, farRight};
        }

    private:
        break_points_type BreakPoints;
        const std::vector<Vertex2D>& Sites;
        SiteEventId InitialSite;
    };
} // namespace compg