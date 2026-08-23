#pragma once
#include <utility>

#include "DoublyConnectedEdgeList.hpp"
#include "math/Geometry.hpp"
#include "math/primitives/Box.hpp"
#include "math/primitives/HalfLine.hpp"
#include "math/primitives/HalfPlane.hpp"
#include "math/primitives/LineSegment.hpp"

namespace compg {
    class Cropper2D {
    public:
        explicit Cropper2D(Box2D box)
            : Box(std::move(box)) {}

        /**
         * @brief Intersect a line segment with a half plane.
         * @param segment A line segment.
         * @param halfPlane A half plane.
         * @return The intersected line segment if it is not empty, otherwise std::nullopt.
         */
        template <typename HyperplaneType>
        static std::optional<LineSegment2D>
        Crop(const LineSegment2D& segment, const HalfPlane<HyperplaneType>& halfPlane) {
            const auto s0 = FindSide<2UZ>(halfPlane, segment[0]);
            const auto s1 = FindSide<2UZ>(halfPlane, segment[1]);

            if (s0 != PointSide::Negative && s1 != PointSide::Negative) {
                return segment;
            }

            if (s0 == PointSide::Negative && s1 == PointSide::Negative) {
                return std::nullopt;
            }

            const auto intersection = FindIntersection(halfPlane.Plane, segment);
            COMPG_ASSERT(intersection.has_value(), "Expected the segment to intersect the half plane");
            if (s0 == PointSide::Negative) {
                return LineSegment2D{intersection.value(), segment[1]};
            }
            return LineSegment2D{segment[0], intersection.value()};
        }

        /**
         * @brief Intersect a line segment with multiple half planes.
         */
        template <typename HyperplaneType>
        static std::optional<LineSegment2D>
        Crop(const LineSegment2D& segment, const std::vector<HalfPlane<HyperplaneType>>& halfPlanes) {
            std::optional result{segment};
            for (std::size_t i = 0; i < halfPlanes.size() && result.has_value(); ++i) {
                result = Crop(result.value(), halfPlanes.at(i));
            }
            return result;
        }

        /**
         * @brief Bound and intersect a half line with a half plane.
         * @param halfLine A half line.
         * @param halfPlane A half plane such that its intersection with the half line is bounded.
         * @return The line segment that is their intersection if it is not empty, otherwise std::nullopt.
         */
        static std::optional<LineSegment2D>
        Bound(const HalfLine2D& halfLine, const HalfPlane<AxesAlignedHyperplane<2UZ>>& halfPlane);

        /**
         * @brief Bound a half line with the box.
         */
        [[nodiscard]] std::optional<LineSegment2D> Bound(const HalfLine2D& halfLine) const;

        /**
         * @brief Intersect a line segment with the box.
         * @param segment A line segment.
         * @return The intersection if it is not empty, otherwise std::nullopt.
         */
        [[nodiscard]] std::optional<LineSegment2D> Crop(const LineSegment2D& segment) const;

        /**
         * @brief Intersect a half line with the box.
         * @param halfLine A half line.
         * @return The intersection if it is not empty, otherwise std::nullopt.
         */
        [[nodiscard]] std::optional<LineSegment2D> Crop(const HalfLine2D& halfLine) const;

        /**
         * @brief Crop a subdivision of the plane induced by a set of line segments and half lines.
         */
        DoublyConnectedEdgeList
        Crop(const std::vector<LineSegment2D>& segments, const std::vector<HalfLine2D>& halfLines) const;

    private:
        auto GetSides() const {
            const Vertex2D& v0 = Box.GetLowerCorner();
            const Vertex2D& v2 = Box.GetUpperCorner();
            const std::array<HalfPlane<AxesAlignedHyperplane<2UZ>>, 4> sides{
                {{{0, v0[0]}, Side::Positive},
                 {{1, v0[1]}, Side::Positive},
                 {{0, v2[0]}, Side::Negative},
                 {{1, v2[1]}, Side::Negative}}
            };
            return sides;
        }

    private:
        Box2D Box;
    };
} // namespace compg
