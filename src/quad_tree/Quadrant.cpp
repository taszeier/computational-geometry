#include "quad_tree/Quadrant.hpp"

namespace compg {
    Box2D CreateNorthEastQuadrant(const Box2D& box) {
        return {box.GetCenter(), box.GetUpperCorner()};
    }

    Box2D CreateNorthWestQuadrant(const Box2D& box) {
        const auto center = box.GetCenter();
        auto corner = box.GetLowerCorner();
        corner(1) += box.GetSideLength(1);
        return {corner, center};
    }

    Box2D CreateSouthWestQuadrant(const Box2D& box) {
        return {box.GetLowerCorner(), box.GetCenter()};
    }

    Box2D CreateSouthEastQuadrant(const Box2D& box) {
        const auto center = box.GetCenter();
        auto corner = box.GetLowerCorner();
        corner(0) += box.GetSideLength(0);
        return {center, corner};
    }

    Box2D CreateQuadrant(const Box2D& box, OrdinalDirection d) {
        switch (d) {
        case OrdinalDirection::NorthEast:
            return CreateNorthEastQuadrant(box);
        case OrdinalDirection::NorthWest:
            return CreateNorthWestQuadrant(box);
        case OrdinalDirection::SouthWest:
            return CreateSouthWestQuadrant(box);
        case OrdinalDirection::SouthEast:
            return CreateSouthEastQuadrant(box);
        }
        COMPG_THROW("Received unexpected ordinal direction");
    }

    OrdinalDirection FindQuadrantOfVertex(const Vertex2D& center, const Vertex2D& vertex) {
        if (center.x() < vertex.x() && center.y() < vertex.y())
            return OrdinalDirection::NorthEast;
        if (center.x() >= vertex.x() && center.y() < vertex.y())
            return OrdinalDirection::NorthWest;
        if (center.x() >= vertex.x() && center.y() >= vertex.y())
            return OrdinalDirection::SouthWest;
        return OrdinalDirection::SouthEast;
    }
} // namespace compg