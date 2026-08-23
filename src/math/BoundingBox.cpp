#include "math/BoundingBox.hpp"

namespace compg {
    Box2D FindBoundingBox(const LineSegment2D& segment) {
        return Box2D{segment[0], segment[1]};
    }

    Box3D FindBoundingBox(const Triangle3D& triangle) {
        const Float xMin = std::min(std::min(triangle[0][0], triangle[1][0]), triangle[2][0]);
        const Float xMax = std::max(std::max(triangle[0][0], triangle[1][0]), triangle[2][0]);

        const Float yMin = std::min(std::min(triangle[0][1], triangle[1][1]), triangle[2][1]);
        const Float yMax = std::max(std::max(triangle[0][1], triangle[1][1]), triangle[2][1]);

        const Float zMin = std::min(std::min(triangle[0][2], triangle[1][2]), triangle[2][2]);
        const Float zMax = std::max(std::max(triangle[0][2], triangle[1][2]), triangle[2][2]);

        return Box3D{{xMin, yMin, zMin}, {xMax, yMax, zMax}};
    }
} // namespace compg