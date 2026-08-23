#pragma once

#include "math/primitives/Box.hpp"
#include "quad_tree/Directions.hpp"

namespace compg {
    Box2D CreateNorthEastQuadrant(const Box2D& box);
    Box2D CreateNorthWestQuadrant(const Box2D& box);
    Box2D CreateSouthWestQuadrant(const Box2D& box);
    Box2D CreateSouthEastQuadrant(const Box2D& box);
    Box2D CreateQuadrant(const Box2D& box, OrdinalDirection d);

    OrdinalDirection FindQuadrantOfVertex(const Vertex2D& center, const Vertex2D& vertex);
} // namespace compg