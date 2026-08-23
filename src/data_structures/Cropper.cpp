#include "data_structures/Cropper.hpp"

#include "data_structures/AvlTree.hpp"
#include "math/Conversions.hpp"
#include "math/Geometry.hpp"

namespace compg {
    std::optional<LineSegment2D> Cropper2D::Crop(const LineSegment2D& segment) const {
        const auto sides = GetSides();
        std::optional result = segment;
        for (std::size_t i = 0; i < sides.size() && result.has_value(); ++i) {
            result = Crop(result.value(), sides.at(i));
        }

        return result;
    }

    std::optional<LineSegment2D>
    Cropper2D::Bound(const HalfLine2D& halfLine, const HalfPlane<AxesAlignedHyperplane<2UZ>>& halfPlane) {
        const auto side = FindSide<2UZ>(halfPlane, halfLine.GetOrigin());
        if (side == PointSide::Negative) {
            return std::nullopt;
        }

        const auto v1 = FindIntersection(halfLine, ConvertTo<Line2D>(halfPlane.Plane));
        COMPG_ASSERT(v1.has_value(), "Expected the half line to intersect the half plane boundary");
        return LineSegment2D{halfLine.GetOrigin(), v1.value()};
    }

    std::optional<LineSegment2D> Cropper2D::Bound(const HalfLine2D& halfLine) const {
        const auto angle = Angle360({-1, 1}, halfLine.GetNormal());
        constexpr std::array thresholds{math::PI * 0.5, math::PI, math::PI * 1.5, math::PI * 2};
        std::size_t i = 0;
        while (i < thresholds.size() && thresholds.at(i) < angle) {
            ++i;
        }
        const auto sides = GetSides();

        return Bound(halfLine, sides.at(i));
    }

    std::optional<LineSegment2D> Cropper2D::Crop(const HalfLine2D& halfLine) const {
        auto result = Bound(halfLine);
        const auto sides = GetSides();
        for (std::size_t i = 0; i < sides.size() && result.has_value(); ++i) {
            result = Crop(result.value(), sides.at(i));
        }
        return result;
    }

    DoublyConnectedEdgeList
    Cropper2D::Crop(const std::vector<LineSegment2D>& segments, const std::vector<HalfLine2D>& halfLines) const {
        // TODO: refactor this mess.
        DoublyConnectedEdgeList result;
        const Vertex2D& v0 = Box.GetLowerCorner();
        const Vertex2D& v2 = Box.GetUpperCorner();
        const Vertex2D v1{v2.x(), v0.y()};
        const Vertex2D v3{v0.x(), v2.y()};

        const LineSegment2D vert1{v0, v3};
        const LineSegment2D vert2{v1, v2};
        const LineSegment2D hor1{v3, v2};
        const LineSegment2D hor2{v0, v1};

        auto xCoord = [&result](DoublyConnectedEdgeList::vertex_index v) { return result.GetVertex(v).Vertex[0]; };
        auto yCoord = [&result](DoublyConnectedEdgeList::vertex_index v) { return result.GetVertex(v).Vertex[1]; };

        AvlTree<DoublyConnectedEdgeList::vertex_index> vert1Indices;
        AvlTree<DoublyConnectedEdgeList::vertex_index> vert2Indices;
        AvlTree<DoublyConnectedEdgeList::vertex_index> hor1Indices;
        AvlTree<DoublyConnectedEdgeList::vertex_index> hor2Indices;

        {
            const auto i0 = result.InsertVertex(v0);
            const auto i1 = result.InsertVertex(v1);
            const auto i2 = result.InsertVertex(v2);
            const auto i3 = result.InsertVertex(v3);
            vert1Indices.Insert(i0, Less{}, yCoord);
            vert1Indices.Insert(i3, Less{}, yCoord);
            vert2Indices.Insert(i1, Less{}, yCoord);
            vert2Indices.Insert(i2, Less{}, yCoord);

            hor1Indices.Insert(i3, Less{}, xCoord);
            hor1Indices.Insert(i2, Less{}, xCoord);
            hor2Indices.Insert(i0, Less{}, xCoord);
            hor2Indices.Insert(i1, Less{}, xCoord);
        }
        auto handleVertex = [&](DoublyConnectedEdgeList::vertex_index v) {
            if (FindSide(vert1, result.GetVertex(v).Vertex) == PointSide::Collinear) {
                vert1Indices.Insert(v, Less{}, yCoord);
            }
            if (FindSide(vert2, result.GetVertex(v).Vertex) == PointSide::Collinear) {
                vert2Indices.Insert(v, Less{}, yCoord);
            }
            if (FindSide(hor1, result.GetVertex(v).Vertex) == PointSide::Collinear) {
                hor1Indices.Insert(v, Less{}, xCoord);
            }
            if (FindSide(hor2, result.GetVertex(v).Vertex) == PointSide::Collinear) {
                hor2Indices.Insert(v, Less{}, xCoord);
            }
        };

        for (const auto& segment : segments) {
            const auto maybeSegment = Crop(segment);
            if (maybeSegment.has_value() && !AreEqual(maybeSegment.value()[0], maybeSegment.value()[1])) {
                const auto i0 = result.InsertVertex(maybeSegment.value()[0]);
                const auto i1 = result.InsertVertex(maybeSegment.value()[1]);
                result.InsertEdge(i0, i1);
                handleVertex(i0);
                handleVertex(i1);
            }
        }

        for (const auto& halfLine : halfLines) {
            ;
            const auto maybeSegment = Crop(halfLine);
            if (maybeSegment.has_value() && !AreEqual(maybeSegment.value()[0], maybeSegment.value()[1])) {
                const auto ii0 = result.InsertVertex(maybeSegment.value()[0]);
                const auto ii1 = result.InsertVertex(maybeSegment.value()[1]);
                result.InsertEdge(ii0, ii1);
                handleVertex(ii0);
                handleVertex(ii1);
            }
        }

        auto addEdges = [&](auto& vertices) {
            auto prev = vertices.begin();
            auto it = std::next(prev);
            while (it != vertices.end()) {
                result.InsertEdge(*prev, *it);
                prev = it;
                ++it;
            }
        };

        addEdges(vert1Indices);
        addEdges(vert2Indices);
        addEdges(hor1Indices);
        addEdges(hor2Indices);

        return result;
    }
} // namespace compg
