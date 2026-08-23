#include "linear_programming/MinimumDiscCalculator.hpp"
#include "math/Geometry.hpp"

namespace compg {

    namespace details {
        Ball<2UZ> CreateMinimumDisc(const Vertex2D& v1, const Vertex2D& v2) {
            const auto center = (v1 + v2) * 0.5;
            const auto radius = (v1 - v2).norm() * 0.5;
            return Ball<2UZ>{center, radius};
        }

        Ball<2UZ> CreateMinimumDisc(const Vertex2D& v1, const Vertex2D& v2, const Vertex2D& v3) {
            const auto bisector12 = CreateBisector(v1, v2);
            const auto bisector13 = CreateBisector(v1, v3);
            const auto circumcenter = FindIntersection(bisector12, bisector13);
            if (!circumcenter.has_value()) {
                // collinear
                const std::array<Float, 3> distances{
                    (v1 - v2).squaredNorm(), (v1 - v3).squaredNorm(), (v2 - v3).squaredNorm()
                };
                const std::array pairs{std::tuple{v1, v2}, std::tuple{v1, v3}, std::tuple{v2, v3}};
                const auto maxIndex = std::distance(distances.begin(), std::ranges::max_element(distances));
                const auto& maxPair = pairs.at(maxIndex);
                return CreateMinimumDisc(std::get<0>(maxPair), std::get<1>(maxPair));
            }

            const auto radius = (v1 - circumcenter.value()).norm();
            return Ball<2UZ>{circumcenter.value(), radius};
        }

        Ball<2UZ> FindMinimumDisc(
            const std::ranges::range auto& vertices, const Vertex2D& boundaryVertex1, const Vertex2D& boundaryVertex2
        ) {
            auto disc = CreateMinimumDisc(boundaryVertex1, boundaryVertex2);
            std::ranges::for_each(vertices, [&boundaryVertex1, &boundaryVertex2, &disc](const auto& vertex) {
                if (!disc.Contains(vertex)) {
                    disc = CreateMinimumDisc(vertex, boundaryVertex1, boundaryVertex2);
                }
            });
            return disc;
        }

        Ball<2UZ> FindMinimumDisc(const std::ranges::range auto& vertices, const Vertex2D& boundaryVertex) {
            COMPG_ASSERT(vertices.begin() != vertices.end(), "Expect a non-empty range");

            auto disc = CreateMinimumDisc(*vertices.begin(), boundaryVertex);
            for (auto it = std::next(vertices.begin()); it != vertices.end(); ++it) {
                const auto& vertex = *it;
                if (!disc.Contains(vertex)) {
                    disc = FindMinimumDisc(std::ranges::subrange(vertices.begin(), it), boundaryVertex, vertex);
                }
            }
            return disc;
        }
    } // namespace details

    Ball<2UZ> MinimumDiscCalculator::FindMinimumDisc(const std::vector<Vertex2D>& vertices, std::size_t seed) const {
        COMPG_ASSERT(vertices.size() > 1, "Expected at least two vertices");
        auto permutation{vertices};
        RandomPermutation(permutation, seed);

        auto disc = details::CreateMinimumDisc(permutation.at(0), permutation.at(1));
        for (auto it = std::next(permutation.begin(), 2); it != permutation.end(); ++it) {
            const auto& vertex = *it;
            if (!disc.Contains(vertex)) {
                disc = details::FindMinimumDisc(std::ranges::subrange(permutation.begin(), it), vertex);
            }
        }

        return disc;
    }
} // namespace compg