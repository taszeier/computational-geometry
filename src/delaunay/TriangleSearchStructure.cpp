#include "delaunay/TriangleSearchStructure.hpp"

namespace compg {

    TriangleSearchStructure::TriangleSearchStructure(const IndexTriangle& rootTriangle) {
        Root = Nodes.size();
        Nodes.emplace_back(rootTriangle);
        NodeMap.insert(std::pair{rootTriangle, Root});
    }

    void TriangleSearchStructure::OnCenterSplit(const IndexTriangle& triangle, VertexIndex vertex) {
        COMPG_ASSERT(NodeMap.contains(triangle), "The triangle is not in the search structure");
        COMPG_ASSERT(
            Nodes.at(NodeMap.at(triangle)).IsLeaf(), "Expected a triangle that is currently in the triangulation"
        );

        const IndexTriangle tri0{triangle[0], triangle[1], vertex};
        const IndexTriangle tri1{triangle[1], triangle[2], vertex};
        const IndexTriangle tri2{triangle[2], triangle[0], vertex};

        const std::size_t idx0 = Nodes.size();
        Nodes.emplace_back(tri0);
        const std::size_t idx1 = Nodes.size();
        Nodes.emplace_back(tri1);
        const std::size_t idx2 = Nodes.size();
        Nodes.emplace_back(tri2);

        const auto parentIndex = NodeMap.at(triangle);
        Nodes.at(parentIndex).Children.at(0) = idx0;
        Nodes.at(parentIndex).Children.at(1) = idx1;
        Nodes.at(parentIndex).Children.at(2) = idx2;

        NodeMap.insert_or_assign(tri0, idx0);
        NodeMap.insert_or_assign(tri1, idx1);
        NodeMap.insert_or_assign(tri2, idx2);
    }

    void TriangleSearchStructure::OnEdgeSplit(const AdjacentTriangles& triangles, VertexIndex vertex) {
        const auto [tri0, tri1] = triangles.GetTriangles();
        COMPG_ASSERT(NodeMap.contains(tri0), "The left triangle is not in the search structure");
        COMPG_ASSERT(Nodes.at(NodeMap.at(tri0)).IsLeaf(), "Expected a triangle that is currently in the triangulation");
        COMPG_ASSERT(NodeMap.contains(tri1), "The right triangle is not in the search structure");
        COMPG_ASSERT(Nodes.at(NodeMap.at(tri1)).IsLeaf(), "Expected a triangle that is currently in the triangulation");

        const auto idx0 = NodeMap.at(tri0);
        const auto idx1 = NodeMap.at(tri1);

        const IndexTriangle tri00{triangles.CommonEdge[0], vertex, triangles.LeftVertex};
        const IndexTriangle tri01{vertex, triangles.CommonEdge[1], triangles.LeftVertex};
        const IndexTriangle tri10{triangles.CommonEdge[0], triangles.RightVertex, vertex};
        const IndexTriangle tri11{triangles.CommonEdge[1], vertex, triangles.RightVertex};

        const std::size_t idx00 = Nodes.size();
        Nodes.emplace_back(tri00);
        const std::size_t idx01 = Nodes.size();
        Nodes.emplace_back(tri01);
        const std::size_t idx10 = Nodes.size();
        Nodes.emplace_back(tri10);
        const std::size_t idx11 = Nodes.size();
        Nodes.emplace_back(tri11);

        Nodes.at(idx0).Children.at(0) = idx00;
        Nodes.at(idx0).Children.at(1) = idx01;
        Nodes.at(idx1).Children.at(0) = idx10;
        Nodes.at(idx1).Children.at(1) = idx11;

        NodeMap.insert_or_assign(tri00, idx00);
        NodeMap.insert_or_assign(tri01, idx01);
        NodeMap.insert_or_assign(tri10, idx10);
        NodeMap.insert_or_assign(tri11, idx11);
    }

    void TriangleSearchStructure::OnEdgeFlip(const AdjacentTriangles& triangles) {
        const auto [tri0, tri1] = triangles.GetTriangles();
        COMPG_ASSERT(NodeMap.contains(tri0), "The left triangle is not in the search structure");
        COMPG_ASSERT(Nodes.at(NodeMap.at(tri0)).IsLeaf(), "Expected a triangle that is currently in the triangulation");
        COMPG_ASSERT(NodeMap.contains(tri1), "The right triangle is not in the search structure");
        COMPG_ASSERT(Nodes.at(NodeMap.at(tri1)).IsLeaf(), "Expected a triangle that is currently in the triangulation");

        const auto idx0 = NodeMap.at(tri0);
        const auto idx1 = NodeMap.at(tri1);

        const IndexTriangle tri2{triangles.LeftVertex, triangles.CommonEdge[0], triangles.RightVertex};
        const IndexTriangle tri3{triangles.LeftVertex, triangles.RightVertex, triangles.CommonEdge[1]};

        const std::size_t idx2 = Nodes.size();
        Nodes.emplace_back(tri2);
        const std::size_t idx3 = Nodes.size();
        Nodes.emplace_back(tri3);

        Nodes.at(idx0).Children.at(0) = idx2;
        Nodes.at(idx0).Children.at(1) = idx3;
        Nodes.at(idx1).Children.at(0) = idx2;
        Nodes.at(idx1).Children.at(1) = idx3;

        NodeMap.insert_or_assign(tri2, idx2);
        NodeMap.insert_or_assign(tri3, idx3);
    }
} // namespace compg