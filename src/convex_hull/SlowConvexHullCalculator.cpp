#include "convex_hull/SlowConvexHullCalculator.hpp"
#include "math/Geometry.hpp"

namespace compg {

    using vertex_type = SlowConvexHullCalculator::vertex_type;

    namespace details {
        SlowConvexHullCalculator::convex_hull_type
        SortClockwise(const std::vector<LineSegment<vertex_type>>& segments) {
            COMPG_ASSERT(!segments.empty(), "Received no segments");

            std::vector<Vertex2D> vertices;
            vertices.reserve(segments.size());
            vertices.push_back(segments.at(0)[0]);

            for ([[maybe_unused]] std::size_t i : std::views::iota(1UZ, segments.size())) {
                auto it = std::ranges::find_if(segments, [&vertices](const LineSegment<vertex_type>& l) {
                    return AreEqual(l[0], vertices.back());
                });

                COMPG_ASSERT(it != segments.end(), "Invalid segments");
                vertices.push_back((*it)[1]);
            }

            return SlowConvexHullCalculator::convex_hull_type{vertices};
        }
    } // namespace details

    SlowConvexHullCalculator::convex_hull_type
    SlowConvexHullCalculator::FindConvexHull(const std::vector<vertex_type>& vertices) const {
        std::vector<LineSegment<vertex_type>> segments;

        for (const auto& [v1, v2] : std::views::cartesian_product(vertices, vertices)) {
            if (!AreEqual(v1, v2, Epsilon) && IsValid(v1, v2, vertices)) {
                segments.emplace_back(v1, v2);
            }
        }
        return details::SortClockwise(segments);
    }

    bool SlowConvexHullCalculator::IsValid(
        const vertex_type& v1, const vertex_type& v2, const std::vector<vertex_type>& vertices
    ) const {
        return std::ranges::none_of(vertices, [this, &v1, &v2](const vertex_type& v) {
            return FindSide(v1, v2, v, Epsilon) == PointSide::Negative;
        });
    }
} // namespace compg