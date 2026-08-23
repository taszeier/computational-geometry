#include "data_structures/DoublyConnectedEdgeList.hpp"
#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"

#include "common/Error.hpp"
#include "math/Geometry.hpp"

namespace compg {
    using vertex_index = DoublyConnectedEdgeList::vertex_index;
    using edge_index = DoublyConnectedEdgeList::edge_index;

    vertex_index DoublyConnectedEdgeList::InsertVertex(const Vertex2D& vertex) {
        if (!VertexIndexMap.contains(vertex)) {
            vertex_index vertexIndex{Vertices.size()};
            Vertices.emplace_back(vertex, std::nullopt);
            VertexIndexMap[vertex] = vertexIndex;
        }

        return VertexIndexMap[vertex];
    }

    bool DoublyConnectedEdgeList::HasEdge(vertex_index v1, vertex_index v2) const {
        return EdgeIndexMap.contains(edge_type{v1, v2});
    }

    bool DoublyConnectedEdgeList::HasVertex(const Vertex2D& vertex) const {
        return VertexIndexMap.contains(vertex);
    }

    edge_index DoublyConnectedEdgeList::InsertEdge(vertex_index v1, vertex_index v2) {
        COMPG_ASSERT(v1 < Vertices.size(), CreateOutOfRangeMessage("v1", v1, Vertices.size()));
        COMPG_ASSERT(v2 < Vertices.size(), CreateOutOfRangeMessage("v2", v2, Vertices.size()));
        COMPG_ASSERT(v1 != v2, "Expected two distinct vertices");

        const edge_type edge{v1, v2};
        if (!EdgeIndexMap.contains(edge)) {
            const edge_index i = Edges.size();
            const edge_index j = Edges.size() + 1;

            const auto v1Next = FindCyclicNext(*this, v2, v1);
            const edge_index e1 = v1Next.value_or(j);
            const edge_index e4 = v1Next.has_value() ? Edges[e1].PreviousIndex : i;

            const auto v2Next = FindCyclicNext(*this, v1, v2);
            const edge_index e3 = v2Next.value_or(i);
            const edge_index e2 = v2Next.has_value() ? Edges[e3].PreviousIndex : j;

            Edges.emplace_back(v2, j, e1, e2, 0);
            Edges.emplace_back(v1, i, e3, e4, 0);

            Edges[e1].PreviousIndex = i;
            Edges[e2].NextIndex = i;
            Edges[e3].PreviousIndex = j;
            Edges[e4].NextIndex = j;

            EdgeIndexMap[edge_type{v2, v1}] = i;
            EdgeIndexMap[edge] = j;

            if (!Vertices[v2].IncidentEdgeIndex.has_value()) {
                Vertices[v2].IncidentEdgeIndex = i;
            }

            if (!Vertices[v1].IncidentEdgeIndex.has_value()) {
                Vertices[v1].IncidentEdgeIndex = j;
            }
        }
        AreFacesValid_ = false;

        return EdgeIndexMap[edge];
    }

    Vertex2D DoublyConnectedEdgeList::GetEdgeAsVector(edge_index edgeIndex) const {
        COMPG_ASSERT(edgeIndex < Edges.size(), CreateOutOfRangeMessage("edgeIndex", edgeIndex, Edges.size()));

        const Vertex2D start = Vertices[Edges[edgeIndex].OriginIndex].Vertex;
        const Vertex2D end = Vertices[Edges[Edges[edgeIndex].TwinIndex].OriginIndex].Vertex;

        return end - start;
    }

    LineSegment2D DoublyConnectedEdgeList::GetEdgeAsLineSegment(edge_index edgeIndex) const {
        COMPG_ASSERT(edgeIndex < Edges.size(), CreateOutOfRangeMessage("edgeIndex", edgeIndex, Edges.size()));
        const Vertex2D start = Vertices[Edges[edgeIndex].OriginIndex].Vertex;
        const Vertex2D end = Vertices[Edges[Edges[edgeIndex].TwinIndex].OriginIndex].Vertex;
        return LineSegment2D{start, end};
    }

    DoublyConnectedEdgeList::vertex_index DoublyConnectedEdgeList::Split(edge_index edgeIndex, const Vertex2D& vertex) {
        COMPG_ASSERT(edgeIndex < NumEdges(), CreateOutOfRangeMessage("edgeIndex", edgeIndex, NumEdges()));
        if (VertexIndexMap.contains(vertex)) {
            const auto vertexIndex = VertexIndexMap.at(vertex);
            const auto originIndex = GetOriginIndex(edgeIndex);
            const auto destinationIndex = GetDestinationIndex(edgeIndex);
            COMPG_ASSERT(
                !EdgeIndexMap.contains(edge_type{originIndex, vertexIndex}), "Split would create a duplicate edge"
            );
            COMPG_ASSERT(
                !EdgeIndexMap.contains(edge_type{destinationIndex, vertexIndex}), "Split would create a duplicate edge"
            );
        }
        COMPG_ASSERT(GetVertex(GetOriginIndex(edgeIndex)).Vertex != vertex, "Cannot split at an endpoint of the edge");
        COMPG_ASSERT(
            GetVertex(GetDestinationIndex(edgeIndex)).Vertex != vertex, "Cannot split at an endpoint of the edge"
        );

        const auto originIndex = GetOriginIndex(edgeIndex);
        const auto destinationIndex = GetDestinationIndex(edgeIndex);

        const auto vertexIndex = InsertVertex(vertex);
        const auto twinIndex = GetTwinIndex(edgeIndex);

        const auto n1 = Edges.size();
        Edges.emplace_back(vertexIndex, twinIndex, GetNextIndex(edgeIndex), edgeIndex, GetFaceIndex(edgeIndex));
        const auto n2 = Edges.size();
        Edges.emplace_back(vertexIndex, edgeIndex, GetNextIndex(twinIndex), twinIndex, GetFaceIndex(twinIndex));

        Edges.at(edgeIndex).NextIndex = n1;
        Edges.at(edgeIndex).TwinIndex = n2;
        if (Edges.at(edgeIndex).PreviousIndex == twinIndex) {
            Edges.at(edgeIndex).PreviousIndex = n2;
        }
        Edges.at(twinIndex).NextIndex = n2;
        Edges.at(twinIndex).TwinIndex = n1;
        if (Edges.at(twinIndex).PreviousIndex == edgeIndex) {
            Edges.at(twinIndex).PreviousIndex = n1;
        }

        Edges.at(GetNextIndex(n1)).PreviousIndex = n1;
        Edges.at(GetNextIndex(n2)).PreviousIndex = n2;

        if (!Vertices.at(vertexIndex).IncidentEdgeIndex.has_value()) {
            Vertices.at(vertexIndex).IncidentEdgeIndex = n1;
        }

        EdgeIndexMap.erase(edge_type{originIndex, destinationIndex});
        EdgeIndexMap.erase(edge_type{destinationIndex, originIndex});
        EdgeIndexMap.insert(std::pair{edge_type{originIndex, vertexIndex}, edgeIndex});
        EdgeIndexMap.insert(std::pair{edge_type{vertexIndex, originIndex}, n2});
        EdgeIndexMap.insert(std::pair{edge_type{destinationIndex, vertexIndex}, twinIndex});
        EdgeIndexMap.insert(std::pair{edge_type{vertexIndex, destinationIndex}, n1});

        return vertexIndex;
    }

    DoublyConnectedEdgeList::edge_index DoublyConnectedEdgeList::SplitFace(vertex_index v1, vertex_index v2) {
        const auto edgeIndex = InsertEdge(v1, v2);
        const auto twinIndex = GetTwinIndex(edgeIndex);

        const auto oldFaceIndex = GetFaceIndex(GetNextIndex(edgeIndex));
        COMPG_ASSERT(
            Faces.at(oldFaceIndex).InnerComponents.empty(), "Expected the face to not have any inner components"
        );
        Edges.at(twinIndex).FaceIndex = oldFaceIndex;

        Faces.at(oldFaceIndex).OuterComponent = twinIndex;
        const DoublyConnectedEdgeList::face_index newFaceIndex = Faces.size();
        Faces.emplace_back(edgeIndex);

        auto e = edgeIndex;
        do {
            Edges.at(e).FaceIndex = newFaceIndex;
            e = GetNextIndex(e);
        } while (e != edgeIndex);

        return edgeIndex;
    }

    void DoublyConnectedEdgeList::CheckIsValid() const {
        // TODO: Add more checks.
        for (std::size_t i = 0; i < Edges.size(); ++i) {
            COMPG_ASSERT(
                Edges.at(Edges.at(i).TwinIndex).TwinIndex == i, "The twin of the twin is not the edge itself."
            );

            COMPG_ASSERT(
                Edges.at(Edges.at(i).NextIndex).PreviousIndex == i,
                "The previous edge of the next edge is not the edge itself."
            );
            COMPG_ASSERT(
                Edges.at(Edges.at(i).PreviousIndex).NextIndex == i,
                "The next edge of the previous edge is not the edge itself."
            );
        }
    }
} // namespace compg
