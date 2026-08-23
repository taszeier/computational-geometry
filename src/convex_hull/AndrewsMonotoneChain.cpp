#include "convex_hull/AndrewsMonotoneChain.hpp"
#include "math/Geometry.hpp"

#include <ranges>

namespace compg {
    using vertex_type = AndrewsMonotoneChain::vertex_type;

    namespace details {
        bool IsRightTurn(const Vertex2D& v1, const Vertex2D& v2, const Vertex2D& v3, Float epsilon) {
            PointSide side = FindSide(v1, v2, v3, epsilon);
            return side == PointSide::Positive;
        }

        std::vector<vertex_type> FindUpperHull(const std::vector<vertex_type>& verticesSorted, Float epsilon) {
            std::vector<vertex_type> upperHull{verticesSorted.at(0), verticesSorted.at(1)};

            for (const vertex_type& v : verticesSorted | std::views::drop(2)) {
                while (upperHull.size() >= 2
                       && !details::IsRightTurn(
                           upperHull.at(upperHull.size() - 2), upperHull.at(upperHull.size() - 1), v, epsilon
                       )) {

                    upperHull.pop_back();
                }
                upperHull.push_back(v);
            }

            return upperHull;
        }

        std::vector<vertex_type> FindLowerHull(const std::vector<vertex_type>& verticesSorted, Float epsilon) {
            std::vector<vertex_type> lowerHull{
                verticesSorted.at(verticesSorted.size() - 1), verticesSorted.at(verticesSorted.size() - 2)
            };

            for (const vertex_type& v : std::views::reverse(verticesSorted) | std::views::drop(2)) {
                while (lowerHull.size() >= 2
                       && !details::IsRightTurn(
                           lowerHull.at(lowerHull.size() - 2), lowerHull.at(lowerHull.size() - 1), v, epsilon
                       )) {

                    lowerHull.pop_back();
                }
                lowerHull.push_back(v);
            }

            return lowerHull;
        }

        std::vector<vertex_type> CombineSemiHulls(
            const std::vector<vertex_type>& upperHull, const std::vector<vertex_type>& lowerHull, Float epsilon
        ) {
            // The vertices are collinear
            if (AreEqual(upperHull, lowerHull, epsilon)) {
                return upperHull;
            }
            COMPG_ASSERT(lowerHull.size() >= 2, "Expected at least two vertices in the lower hull");
            std::vector<vertex_type> hullVertices;
            hullVertices.reserve(upperHull.size() + lowerHull.size() - 2);
            std::ranges::copy(upperHull, std::back_inserter(hullVertices));
            std::copy(lowerHull.begin() + 1, lowerHull.end() - 1, std::back_inserter(hullVertices));

            return hullVertices;
        }

    } // namespace details

    AndrewsMonotoneChain::convex_hull_type
    AndrewsMonotoneChain::FindConvexHull(const std::vector<vertex_type>& vertices) const {
        if (vertices.size() <= 2) {
            return convex_hull_type(vertices);
        }
        std::vector verticesSorted(vertices);
        std::ranges::sort(verticesSorted, LexicographicalLess<vertex_type>());

        const std::vector<vertex_type> upperHull = details::FindUpperHull(verticesSorted, Epsilon);
        const std::vector<vertex_type> lowerHull = details::FindLowerHull(verticesSorted, Epsilon);
        std::vector<vertex_type> hullVertices = details::CombineSemiHulls(upperHull, lowerHull, Epsilon);

        return convex_hull_type(std::move(hullVertices));
    }
} // namespace compg