#include "math/Geometry.hpp"

#include "math/Conversions.hpp"
#include "math/Math.hpp"

namespace compg {
    using namespace math;

    PointSide FindSide(const Vertex2D& a, const Vertex2D& b, const Vertex2D& v, Float epsilon) {
        const Vertex2D ba = b - a;
        const Vertex2D va = v - a;

        Float determinant = Determinant(va, ba);
        if (IsZero(determinant, epsilon)) {
            return PointSide::Collinear;
        }
        if (determinant > 0) {
            return PointSide::Positive;
        }
        return PointSide::Negative;
    }

    PointSide FindSide(const Line2D& line, const Vertex2D& v, Float epsilon) {
        return FindSide(line[0], line[1], v, epsilon);
    }

    PointSide FindSide(const LineSegment2D& segment, const Vertex2D& v, Float epsilon) {
        return FindSide(segment[0], segment[1], v, epsilon);
    }

    PointSide FindSide(const HalfLine2D& halfLine, const Vertex2D& v, Float epsilon) {
        const Vertex2D& v0 = halfLine.GetOrigin();
        const Vertex2D v1 = v0 + halfLine.GetNormal();
        return FindSide(v0, v1, v, epsilon);
    }

    namespace details {
        struct IntersectionParameters {
            Float T, U;
        };

        /**
         * @brief Finds real values t and u such that v1 + t * (v2 - v1) == v3 + u * (v4 -
         * v3), where the point is the intersection of the lines defined by the point pairs
         * (v1, v2) and (v3, v4).
         * @param v1 First point on the first line.
         * @param v2 Second point on the first line.
         * @param v3 First point on the second line.
         * @param v4 Second point on the second line.
         * @param epsilon Precision used when comparing doubles.
         * @return The intersection parameters, t and u, if the lines intersect
         * at a unique point. An empty optional is returned if the lines are
         * parallel or collinear.
         */
        std::optional<IntersectionParameters> FindIntersectionParameters(
            const Vertex2D& v1, const Vertex2D& v2, const Vertex2D& v3, const Vertex2D& v4, Float epsilon = EPSILON
        ) {
            Float determinant = Determinant(v2 - v1, v3 - v4);
            if (!IsZero(determinant, epsilon)) {
                Float t = Cross<Vertex2D>(v4 - v3, v3 - v1) / determinant;
                Float u = Cross<Vertex2D>(v2 - v1, v3 - v1) / determinant;
                return IntersectionParameters{.T = t, .U = u};
            }

            return std::nullopt;
        }

        template <typename L1, typename L2>
        std::optional<IntersectionParameters>
        FindIntersectionParameters(const L1& l1, const L2& l2, Float epsilon = EPSILON) {
            return FindIntersectionParameters(l1[0], l1[1], l2[0], l2[1], epsilon);
        }
    } // namespace details

    std::optional<Vertex2D> FindIntersection(const Line2D& l1, const Line2D& l2, Float epsilon) {
        std::optional<details::IntersectionParameters> parameters
            = details::FindIntersectionParameters(l1, l2, epsilon);
        return parameters.transform([&l1](const auto& p) { return l1.LinearInterpolate(p.T); });
    }

    std::optional<Vertex2D> FindIntersection(const LineSegment2D& l1, const LineSegment2D& l2, Float epsilon) {
        std::optional<details::IntersectionParameters> parameters
            = details::FindIntersectionParameters(l1, l2, epsilon);
        return parameters.and_then([&l1](const auto& p) -> std::optional<Vertex2D> {
            if (0 <= p.T && p.T <= 1 && 0 <= p.U && p.U <= 1) {
                return std::make_optional(l1.LinearInterpolate(p.T));
            }
            return std::nullopt;
        });
    }

    std::optional<Vertex2D> FindIntersection(const LineSegment2D& segment, const Line2D& line, Float epsilon) {
        return FindIntersection(line, segment, epsilon);
    }

    std::optional<Vertex2D> FindIntersection(const Line2D& line, const LineSegment2D& segment, Float epsilon) {
        std::optional<details::IntersectionParameters> parameters
            = details::FindIntersectionParameters(segment, line, epsilon);
        return parameters.and_then([&segment](const auto& p) -> std::optional<Vertex2D> {
            if (0 <= p.T && p.T <= 1) {
                return std::make_optional(segment.LinearInterpolate(p.T));
            }
            return std::nullopt;
        });
    }

    std::optional<Vertex2D>
    FindIntersection(const AxesAlignedHyperplane<2UZ>& hl, const LineSegment2D& segment, Float epsilon) {
        return FindIntersection(ConvertTo<Line2D>(hl), segment, epsilon);
    }

    std::optional<Vertex2D> FindIntersection(const Hyperplane2D& hl, const LineSegment2D& segment, Float epsilon) {
        return FindIntersection(ConvertTo<Line2D>(hl), segment, epsilon);
    }

    std::optional<Vertex2D> FindIntersection(const Hyperplane<2UZ>& h1, const Hyperplane<2UZ>& h2, Float epsilon) {
        const auto l1 = ConvertTo<Line2D>(h1);
        const auto l2 = ConvertTo<Line2D>(h2);
        return FindIntersection(l1, l2, epsilon);
    }

    std::optional<Vertex2D> FindIntersection(const HalfLine2D& hl, const Line2D& l, Float epsilon) {
        std::optional<details::IntersectionParameters> parameters
            = details::FindIntersectionParameters(hl.GetOrigin(), hl.GetOrigin() + hl.GetNormal(), l[0], l[1], epsilon);
        return parameters.and_then([&hl](const auto& p) {
            return 0 <= p.T ? std::optional{hl.LinearInterpolate(p.T)} : std::nullopt;
        });
    }

    std::optional<Vertex2D> FindIntersection(const Line2D& l, const HalfLine2D& hl, Float epsilon) {
        return FindIntersection(hl, l, epsilon);
    }

    std::optional<Vertex2D> FindIntersection(const HalfLine2D& hl, const LineSegment2D& s, Float epsilon) {
        std::optional<details::IntersectionParameters> parameters
            = details::FindIntersectionParameters(hl.GetOrigin(), hl.GetOrigin() + hl.GetNormal(), s[0], s[1], epsilon);
        return parameters.and_then([epsilon, &hl](const auto& p) {
            return (-epsilon <= p.T && -epsilon <= p.U && p.U <= 1 + epsilon) ? std::optional{hl.LinearInterpolate(p.T)}
                                                                              : std::nullopt;
        });
    }

    std::optional<Vertex2D> FindIntersection(const LineSegment2D& s, const HalfLine2D& hl, Float epsilon) {
        return FindIntersection(hl, s, epsilon);
    }

    std::optional<Vertex2D> FindIntersection(const HalfLine2D& hl1, const HalfLine2D& hl2, Float epsilon) {
        std::optional<details::IntersectionParameters> parameters = details::FindIntersectionParameters(
            hl1.GetOrigin(), hl1.GetOrigin() + hl1.GetNormal(), hl2.GetOrigin(), hl2.GetOrigin() + hl2.GetNormal(),
            epsilon
        );
        return parameters.and_then([&hl1, epsilon](const auto& p) {
            return (-epsilon <= p.T && -epsilon <= p.U) ? std::optional{hl1.LinearInterpolate(p.T)} : std::nullopt;
        });
    }

    std::optional<Vertex3D>
    FindIntersection(const Hyperplane3D& hyperplane, const Vertex3D& a, const Vertex3D& b, Float epsilon) {
        const Float e = hyperplane.GetNormal().dot(b - a);
        if (math::IsZero(e, epsilon)) {
            return std::nullopt;
        }
        const Float d = hyperplane.GetNormal().dot(a - hyperplane.GetOrigin());
        const Float t = -d / e;
        if (-epsilon <= t && t < 1.0 + epsilon) {
            return math::LinearInterpolate(a, b, t);
        }
        return std::nullopt;
    }

    Float Angle180(const Vertex2D& v1, const Vertex2D& v2) {
        const auto det = Determinant(v1, v2);
        const auto dot = Dot(v1, v2);
        return std::atan2(det, dot);
    }

    Float Angle360(const Vertex2D& v1, const Vertex2D& v2) {
        const auto det = Determinant(v1, v2);
        const auto dot = Dot(v1, v2);
        const auto angle = std::atan2(-det, -dot) + math::PI;
        return angle;
    }

    Vertex2D Rotate(const Vertex2D& v, Float angle) {
        const Float c = std::cos(angle);
        const Float s = std::sin(angle);
        return {v[0] * c - v[1] * s, v[0] * s + v[1] * c};
    }

    Hyperplane<2UZ> Rotate(const Hyperplane<2UZ>& hyperplane, Float angle) {
        return {Rotate(hyperplane.GetOrigin(), angle), Rotate(hyperplane.GetNormal(), angle)};
    }

    Float FindTriangleArea(const Vertex2D& v0, const Vertex2D& v1, const Vertex2D& v2) {
        return 0.5 * std::abs(v0[0] * (v1[1] - v2[1]) + v1[0] * (v2[1] - v0[1]) + v2[0] * (v0[1] - v1[1]));
    }

    Float FindConvexPolygonAreaUnordered(const std::vector<Vertex2D>& vertices) {
        COMPG_ASSERT(!vertices.empty(), "Expected at least one vertex");

        const Vertex2D centroid = std::ranges::fold_left(vertices, Vertex2D{0, 0}, std::plus<Vertex2D>{})
                                  / static_cast<Float>(vertices.size());
        auto verticesSorted{vertices};
        std::ranges::sort(verticesSorted, Less{}, [centroid](const auto& v) { return Angle360({1, 0}, v - centroid); });

        return FindConvexPolygonArea(verticesSorted);
    }

    Float FindConvexPolygonArea(const std::vector<Vertex2D>& vertices) {
        COMPG_ASSERT(!vertices.empty(), "Expected at least one vertex");

        const auto& v0 = vertices.at(0);
        return std::ranges::fold_left(
            vertices | std::views::drop(1) | std::views::slide(2) | std::views::transform([&v0](auto&& iterable) {
                const auto& v1 = *iterable.begin();
                const auto& v2 = *std::next(iterable.begin());
                return FindTriangleArea(v0, v1, v2);
            }),
            0.0, std::plus{}
        );
    }

    bool Contains(const Line3D& line, const Vertex3D& v, Float epsilon) {
        return Contains(line[0], line[1], v, epsilon);
    }

    bool Contains(const Vertex3D& a, const Vertex3D& b, const Vertex3D& v, Float epsilon) {
        const Vertex3D cross = math::Cross(a, b, v);
        return math::IsZero(cross, epsilon);
    }
} // namespace compg
