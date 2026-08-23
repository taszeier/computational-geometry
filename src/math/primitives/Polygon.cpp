#include "math/primitives/Polygon.hpp"
#include "math/Geometry.hpp"

#include <ranges>

namespace compg {
    Polygon Optimize(const Polygon& polygon) {
        std::vector vertices{polygon.GetVertex(0), polygon.GetVertex(1)};

        for (const auto& v2 : polygon.GetVertices() | std::views::drop(2)) {
            const auto& v1 = vertices.back();
            const auto& v0 = vertices.at(vertices.size() - 2);
            if (FindSide(v0, v1, v2) == PointSide::Collinear) {
                vertices.pop_back();
            }
            vertices.push_back(v2);
        }

        COMPG_ASSERT(vertices.size() >= 3, "Received an invalid polygon");
        if (FindSide(vertices.at(vertices.size() - 2), vertices.back(), vertices.at(0)) == PointSide::Collinear) {
            vertices.pop_back();
        }

        COMPG_ASSERT(vertices.size() >= 3, "Received an invalid polygon");
        if (FindSide(vertices.back(), vertices.at(0), vertices.at(1)) == PointSide::Collinear) {
            vertices.erase(vertices.begin());
        }

        return Polygon{vertices};
    }

    namespace details {
        bool AreEqualOffset(const Polygon& polygon1, const auto& vertices2, std::size_t offset2, Float epsilon) {
            for (std::size_t i = 0; i < polygon1.NumVertices(); ++i) {
                const auto& v1 = polygon1.GetVertex(i);
                const auto j = (i + offset2) % vertices2.size();
                const auto& v2 = vertices2.at(j);
                if (!AreEqual(v1, v2, epsilon)) {
                    return false;
                }
            }
            return true;
        }
    } // namespace details

    bool AreEqual(const Polygon& polygon1, const Polygon& polygon2, Float epsilon) {
        if (polygon1.NumVertices() != polygon2.NumVertices()) {
            return false;
        }

        const auto& vertices2 = polygon2.GetVertices();

        auto equals = [epsilon, v0 = polygon1.GetVertex(0)](const auto& v) { return AreEqual(v, v0, epsilon); };
        auto it = std::ranges::find_if(vertices2, equals);
        bool equal = false;

        while (it != vertices2.end() && !equal) {
            const auto offset2 = std::distance(vertices2.begin(), it);
            equal = details::AreEqualOffset(polygon1, vertices2, offset2, epsilon);

            it = std::ranges::find_if(std::next(it), vertices2.end(), equals);
        }

        return equal;
    }
} // namespace compg