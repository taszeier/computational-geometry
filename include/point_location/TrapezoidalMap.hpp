#pragma once

#include <unordered_set>

#include "common/Vertex.hpp"
#include "math/primitives/Box.hpp"
#include "math/primitives/LineSegment.hpp"

namespace compg {

    class TrapezoidalMap {
    public:
        using trapezoid_index = std::size_t;

        struct TrapezoidRecord {
            Vertex2D LeftPoint;
            Vertex2D RightPoint;
            LineSegment2D TopSegment;
            LineSegment2D BottomSegment;

            std::optional<trapezoid_index> UpperLeft = std::nullopt;
            std::optional<trapezoid_index> LowerLeft = std::nullopt;
            std::optional<trapezoid_index> UpperRight = std::nullopt;
            std::optional<trapezoid_index> LowerRight = std::nullopt;
        };

        /**
         * @param boundingBox A bounding box for the line segments used to create the initial trapezoid.
         */
        explicit TrapezoidalMap(const Box2D& boundingBox);

        std::size_t NumTrapezoids() const {
            return Trapezoids.size();
        }

        auto GetTrapezoid(trapezoid_index trapezoidIndex) const {
            COMPG_ASSERT(
                trapezoidIndex < NumTrapezoids(),
                CreateOutOfRangeMessage("trapezoidIndex", trapezoidIndex, NumTrapezoids())
            );
            return Trapezoids.at(trapezoidIndex);
        }

        auto GetBoundingBox() const {
            return BoundingBox;
        }

        /**
         * @brief The result of a split.
         * @details Contains the indices of the new top and bottom trapezoids created for each trapezoid intersected by
         * the line segment, along with optional trapezoid indices for the left and right trapezoids if the line
         * segment starts or ends inside the first or last trapezoid.
         */
        struct SplitResult {
            std::vector<std::tuple<trapezoid_index, trapezoid_index>> TrapezoidIndices;
            std::optional<trapezoid_index> LeftTrapezoidIndex;
            std::optional<trapezoid_index> RightTrapezoidIndex;
        };

        /**
         * @brief Split a trapezoid intersected by a line segment.
         * @param segment The line segment contained inside a trapezoid.
         * @param trapezoidIndex The index of the trapezoid.
         * @return The result of the split.
         */
        SplitResult Split(const LineSegment2D& segment, trapezoid_index trapezoidIndex);

        /**
         * @brief Split the trapezoids intersected by a line segment.
         * @param segment The line segment.
         * @param trapezoidIndices The indices of the trapezoids intersected by the line segment, ordered from left to
         * right.
         * @return The result of the split.
         */
        SplitResult Split(const LineSegment2D& segment, const std::vector<trapezoid_index>& trapezoidIndices);

    private:
        std::vector<TrapezoidRecord> Trapezoids;
        std::unordered_set<Vertex2D> Vertices;
        Box2D BoundingBox;
    };
} // namespace compg
