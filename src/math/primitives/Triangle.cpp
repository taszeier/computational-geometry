#include "math/primitives/Triangle.hpp"
#include "math/Geometry.hpp"

namespace compg {
    bool Contains(const Triangle2D& triangle, const Vertex2D& vertex, Float epsilon) {
        return FindSide(triangle[0], triangle[1], vertex, epsilon) != PointSide::Positive
               && FindSide(triangle[1], triangle[2], vertex, epsilon) != PointSide::Positive
               && FindSide(triangle[2], triangle[0], vertex, epsilon) != PointSide::Positive;
    }
} // namespace compg