#pragma once

#include "common/Common.hpp"
#include "common/Vertex.hpp"
#include "data_structures/graph/DirectedEdge.hpp"
#include <unordered_map>
#include <unordered_set>

#include "math/primitives/LineSegment.hpp"

namespace compg {
    struct UnsafeUnionResult;

    class DoublyConnectedEdgeList {
    public:
        using edge_index = std::size_t;
        using vertex_index = std::size_t;
        using face_index = std::size_t;
        using edge_type = DirectedEdge<vertex_index>;

    public:
        struct VertexRecord {
            Vertex2D Vertex;
            std::optional<edge_index> IncidentEdgeIndex;
        };

        struct FaceRecord {
            std::optional<edge_index> OuterComponent;
            std::vector<edge_index> InnerComponents;
        };

        struct EdgeRecord {
            vertex_index OriginIndex;
            edge_index TwinIndex;
            edge_index NextIndex;
            edge_index PreviousIndex;
            face_index FaceIndex;
        };

    public:
        /**
         * @brief Insert a vertex into the edge list. The edge list does not change if the vertex is a duplicate.
         * @param vertex The vertex to insert.
         * @return The index of the vertex.
         */
        vertex_index InsertVertex(const Vertex2D& vertex);

        /**
         * @brief Insert an edge between two distinct vertices of the edge list.
         * @details The edge should not intersect any other edge of the edge list in its interior. Loops are not
         * allowed, and the edge list does not change if the edge is already present.
         * @param v1 An index of a vertex in the edge list.
         * @param v2 An index of a vertex in the edge list.
         * @return The index of the half edge from v1 to v2.
         */
        edge_index InsertEdge(vertex_index v1, vertex_index v2);

        /**
         * @param vertex A point in the plane.
         * @return Whether the point is a vertex of the edge list.
         */
        bool HasVertex(const Vertex2D& vertex) const;

        /**
         * @param v1 An index of a vertex in the edge list.
         * @param v2 An index of a vertex in the edge list.
         * @return Whether there is an edge between v1 and v2.
         */
        bool HasEdge(vertex_index v1, vertex_index v2) const;

        /**
         * @brief Split an edge into two edges by inserting a vertex separating the endpoints.
         * @details The vertex and the edge must be such that the new edges do not cross any other edges, and if the
         * vertex is already present in the edge list, it must not have any incident edges. Furthermore, the split
         * should not change the inner components of the faces.
         * @param edgeIndex The index of the edge to split.
         * @param vertex The vertex used to split the edge.
         * @return The index of the vertex used to create the split.
         */
        vertex_index Split(edge_index edgeIndex, const Vertex2D& vertex);

        /**
         * @brief Split an edge into two edges by inserting a vertex separating the endpoints.
         * @details The vertex and the edge must be such that the new edges do not cross any other edges, and if the
         * vertex is already present in the edge list, it must not have any incident edges. Furthermore, the split
         * should not change the inner components of the faces.
         * @param v0 The index of an endpoint of the edge.
         * @param v1 The index of the other endpoint of the edge.
         * @param vertex The vertex used to split the edge.
         * @return The index of the vertex used to create the split.
         */
        vertex_index Split(vertex_index v0, vertex_index v1, const Vertex2D& vertex) {
            COMPG_ASSERT(v0 < NumVertices(), CreateOutOfRangeMessage("v0", v0, NumVertices()));
            COMPG_ASSERT(v1 < NumVertices(), CreateOutOfRangeMessage("v1", v1, NumVertices()));
            COMPG_ASSERT(v0 != v1, "Expected distinct vertices");
            COMPG_ASSERT(HasEdge(v0, v1), "The edge list does not contain the edge between the vertices");
            return Split(EdgeIndexMap.at(edge_type{v0, v1}), vertex);
        }

        /**
         * @brief Insert an edge between two vertices that splits the face into two.
         * @details Has the same preconditions as InsertEdge but creates a new face record and updates the face pointers
         * of the edges. The faces have to be valid and the face that is split cannot have inner components.
         * @param v1 An index of a vertex in the edge list.
         * @param v2 An index of a vertex in the edge list.
         * @return The index of the inserted edge.
         */
        edge_index SplitFace(vertex_index v1, vertex_index v2);

        std::size_t NumEdges() const {
            return Edges.size();
        }

        std::size_t NumVertices() const {
            return Vertices.size();
        }

        std::size_t NumFaces() const {
            COMPG_ASSERT(AreFacesValid_, "The face records are invalid.");
            return Faces.size();
        }

        constexpr bool IsFaceBounded(face_index faceIndex) const {
            COMPG_ASSERT(AreFacesValid_, "The face records are invalid.");
            COMPG_ASSERT(faceIndex < NumFaces(), CreateOutOfRangeMessage("faceIndex", faceIndex, NumFaces()));
            return Faces.at(faceIndex).OuterComponent.has_value();
        }

        face_index FindUnboundedFaceIndex() const {
            COMPG_ASSERT(AreFacesValid_, "The face records are invalid.");
            auto faceIndices = std::views::iota(0UZ, NumFaces());
            const auto it = std::ranges::find_if(faceIndices, [this](auto i) { return !IsFaceBounded(i); });
            COMPG_ASSERT(it != faceIndices.end(), "There is no unbounded face");
            return *it;
        }

        auto GetEdge(edge_index edgeIndex) const {
            COMPG_ASSERT(edgeIndex < NumEdges(), CreateOutOfRangeMessage("edgeIndex", edgeIndex, NumEdges()));
            return Edges[edgeIndex];
        }

        auto GetVertex(vertex_index vertexIndex) const {
            COMPG_ASSERT(
                vertexIndex < NumVertices(), CreateOutOfRangeMessage("vertexIndex", vertexIndex, NumVertices())
            );
            return Vertices[vertexIndex];
        }

        auto GetFace(face_index faceIndex) const {
            COMPG_ASSERT(AreFacesValid_, "The face records are invalid.");
            COMPG_ASSERT(faceIndex < NumFaces(), CreateOutOfRangeMessage("faceIndex", faceIndex, NumFaces()));
            return Faces[faceIndex];
        }

        auto GetTwinIndex(edge_index edgeIndex) const {
            COMPG_ASSERT(edgeIndex < NumEdges(), CreateOutOfRangeMessage("edgeIndex", edgeIndex, NumEdges()));
            return Edges[edgeIndex].TwinIndex;
        }

        auto GetOriginIndex(edge_index edgeIndex) const {
            COMPG_ASSERT(edgeIndex < NumEdges(), CreateOutOfRangeMessage("edgeIndex", edgeIndex, NumEdges()));
            return Edges[edgeIndex].OriginIndex;
        }

        auto GetDestinationIndex(edge_index edgeIndex) const {
            COMPG_ASSERT(edgeIndex < NumEdges(), CreateOutOfRangeMessage("edgeIndex", edgeIndex, NumEdges()));
            return Edges[Edges[edgeIndex].TwinIndex].OriginIndex;
        }

        auto GetNextIndex(edge_index edgeIndex) const {
            COMPG_ASSERT(edgeIndex < NumEdges(), CreateOutOfRangeMessage("edgeIndex", edgeIndex, NumEdges()));
            return Edges[edgeIndex].NextIndex;
        }

        auto GetPreviousIndex(edge_index edgeIndex) const {
            COMPG_ASSERT(edgeIndex < NumEdges(), CreateOutOfRangeMessage("edgeIndex", edgeIndex, NumEdges()));
            return Edges[edgeIndex].PreviousIndex;
        }

        auto GetFaceIndex(edge_index edgeIndex) const {
            COMPG_ASSERT(edgeIndex < NumEdges(), CreateOutOfRangeMessage("edgeIndex", edgeIndex, NumEdges()));
            return Edges[edgeIndex].FaceIndex;
        }

        auto GetVertexIndex(const Vertex2D& vertex) const {
            return VertexIndexMap.at(vertex);
        }

        auto GetEdgeIndex(vertex_index v1, vertex_index v2) const {
            return EdgeIndexMap.at(edge_type{v1, v2});
        }

        /**
         * @brief Perform basic checks to see whether the edge list is in a valid state.
         */
        void CheckIsValid() const;

        /**
         * @param edgeIndex The index of a half edge.
         * @return The vector from the origin to the destination of the half edge.
         */
        Vertex2D GetEdgeAsVector(edge_index edgeIndex) const;

        /**
         * @param edgeIndex The index of a half edge.
         * @return A line segment whose endpoints are the same as the endpoints of the half edge.
         */
        LineSegment2D GetEdgeAsLineSegment(edge_index edgeIndex) const;

        constexpr auto AreFacesValid() const {
            return AreFacesValid_;
        }

        struct Isomorphism {
            std::unordered_map<vertex_index, vertex_index> VertexIsomorphism;
            std::unordered_map<edge_index, edge_index> EdgeIsomorphism;
        };

        friend UnsafeUnionResult UnsafeUnion(const DoublyConnectedEdgeList& lhs, const DoublyConnectedEdgeList& rhs);
        friend void UpdateFaces(
            const std::unordered_map<Vertex2D, DoublyConnectedEdgeList::edge_index>& leftNeighborEdgeMap,
            DoublyConnectedEdgeList& edgeList
        );
        friend class OverlayCalculator;
        friend std::optional<Isomorphism>
        FindIsomorphism(const DoublyConnectedEdgeList& from, const DoublyConnectedEdgeList& to, Float epsilon);
        friend void Flip(vertex_index v1, vertex_index v2, DoublyConnectedEdgeList& edgeList);

    private:
        std::vector<VertexRecord> Vertices;
        std::vector<FaceRecord> Faces;
        std::vector<EdgeRecord> Edges;

        std::unordered_map<Vertex2D, vertex_index> VertexIndexMap;
        std::unordered_map<edge_type, edge_index> EdgeIndexMap;
        bool AreFacesValid_{false};
    };
} // namespace compg
