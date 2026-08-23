#pragma once
#include "common/Vertex.hpp"
#include "data_structures/graph/DirectedEdge.hpp"

#include <unordered_set>

namespace compg {
    struct EdgeList3D {
    public:
        using vertex_index = std::size_t;
        using edge_index = std::size_t;
        using face_index = std::size_t;
        using edge_type = DirectedEdge<vertex_index>;

        struct VertexRecord {
            Vertex3D Vertex;
            std::optional<edge_index> IncidentEdge;
        };

        struct FaceRecord {
            edge_index IncidentEdge;
        };

        struct EdgeRecord {
            vertex_index OriginIndex;
            edge_index TwinIndex;
            edge_index NextIndex;
            edge_index PreviousIndex;
            face_index FaceIndex;
        };

        constexpr auto NumVertices() const {
            return Vertices.size();
        }

        constexpr auto NumFaces() const {
            return Faces.size();
        }

        constexpr auto NumEdges() const {
            return Edges.size();
        }

        auto GetVertex(vertex_index vertexIndex) const {
            return Vertices.at(vertexIndex);
        }

        auto GetFace(face_index faceIndex) const {
            return Faces.at(faceIndex);
        }

        auto GetEdge(edge_index edgeIndex) const {
            return Edges.at(edgeIndex);
        }

        vertex_index GetOriginIndex(edge_index edgeIndex) const {
            return Edges.at(edgeIndex).OriginIndex;
        }

        vertex_index GetDestinationIndex(edge_index edgeIndex) const {
            return GetOriginIndex(GetTwinIndex(edgeIndex));
        }

        auto GetOrigin(edge_index edgeIndex) const {
            return GetVertex(GetOriginIndex(edgeIndex));
        }

        edge_index GetTwinIndex(edge_index edgeIndex) const {
            return Edges.at(edgeIndex).TwinIndex;
        }

        edge_index GetNextIndex(edge_index edgeIndex) const {
            return Edges.at(edgeIndex).NextIndex;
        }

        edge_index GetPreviousIndex(edge_index edgeIndex) const {
            return Edges.at(edgeIndex).PreviousIndex;
        }

        face_index GetFaceIndex(edge_index edgeIndex) const {
            return Edges.at(edgeIndex).FaceIndex;
        }

        std::vector<face_index> GetFaceIndices() const {
            return Faces | std::views::keys | std::ranges::to<std::vector>();
        }

        std::vector<vertex_index> GetVertexIndices() const {
            return Vertices | std::views::keys | std::ranges::to<std::vector>();
        }

        std::vector<edge_index> GetEdgeIndices() const {
            return Edges | std::views::keys | std::ranges::to<std::vector>();
        }

    private:
        template <typename IndexType>
        IndexType GetAvailableIndex(const auto& records, auto& erasedIndices) {
            IndexType index{};
            if (!erasedIndices.empty()) {
                const auto it = erasedIndices.begin();
                index = *it;
                erasedIndices.erase(it);
            } else {
                index = records.size();
            }

            COMPG_ASSERT(!records.contains(index), "The index should not be taken.");
            return index;
        }

        auto GetAvailableFaceIndex() {
            return GetAvailableIndex<face_index>(Faces, ErasedFaces);
        }

        auto GetAvailableVertexIndex() {
            return GetAvailableIndex<vertex_index>(Vertices, ErasedVertices);
        }

        auto GetAvailableEdgeIndex() {
            return GetAvailableIndex<edge_index>(Edges, ErasedEdges);
        }

    public:
        vertex_index AddVertex(const Vertex3D& v, std::optional<edge_index> e = std::nullopt) {
            const vertex_index index = GetAvailableVertexIndex();
            Vertices.emplace(index, VertexRecord{v, e});
            return index;
        }

        edge_index InsertEdge(vertex_index v0, vertex_index v1) {
            const auto it = EdgeIndexMap.find(edge_type{v0, v1});
            if (it != EdgeIndexMap.end()) {
                return it->second;
            }

            const edge_index index = GetAvailableEdgeIndex();
            Edges.emplace(index, EdgeRecord{v0, 0, 0, 0, 0});
            EdgeIndexMap[edge_type{v0, v1}] = index;
            if (!Vertices.at(v0).IncidentEdge) {
                Vertices.at(v0).IncidentEdge = index;
            }

            const edge_index twinIndex = GetAvailableEdgeIndex();
            Edges.emplace(twinIndex, EdgeRecord{v1, index, 0, 0, 0});
            EdgeIndexMap[edge_type{v1, v0}] = twinIndex;
            if (!Vertices.at(v1).IncidentEdge) {
                Vertices.at(v1).IncidentEdge = twinIndex;
            }
            Edges.at(index).TwinIndex = twinIndex;

            return index;
        }

        edge_index GetEdgeIndex(vertex_index v0, vertex_index v1) const {
            return EdgeIndexMap.at(edge_type{v0, v1});
        }

        face_index AddFace(edge_index incidentEdge) {
            const face_index index = GetAvailableFaceIndex();
            Faces.emplace(index, FaceRecord{incidentEdge});
            return index;
        }

        void EraseVertex(vertex_index vertexIndex) {
            Vertices.erase(vertexIndex);
            ErasedVertices.insert(vertexIndex);
        }

        void EraseFace(face_index faceIndex) {
            Faces.erase(faceIndex);
            ErasedFaces.insert(faceIndex);
        }

        void EraseEdge(edge_index edgeIndex) {
            const auto v0 = GetOriginIndex(edgeIndex);
            const auto twinIndex = GetTwinIndex(edgeIndex);
            const auto v1 = GetOriginIndex(twinIndex);
            EdgeIndexMap.erase(edge_type{v0, v1});
            EdgeIndexMap.erase(edge_type{v1, v0});

            Edges.erase(twinIndex);
            Edges.erase(edgeIndex);
            ErasedEdges.insert(twinIndex);
            ErasedEdges.insert(edgeIndex);
        }

        void MakeAdjacent(edge_index prev, edge_index next) {
            Edges.at(prev).NextIndex = next;
            Edges.at(next).PreviousIndex = prev;
        }

        struct Isomorphism {
            std::unordered_map<vertex_index, vertex_index> VertexIsomorphism;
            std::unordered_map<edge_index, edge_index> EdgeIsomorphism;
        };

    public:
        std::unordered_map<vertex_index, VertexRecord> Vertices;
        std::unordered_map<face_index, FaceRecord> Faces;
        std::unordered_map<edge_index, EdgeRecord> Edges;

        std::unordered_map<edge_type, edge_index> EdgeIndexMap;
        std::unordered_set<vertex_index> ErasedVertices;
        std::unordered_set<face_index> ErasedFaces;
        std::unordered_set<edge_index> ErasedEdges;
    };

    std::optional<EdgeList3D::Isomorphism>
    FindIsomorphism(const EdgeList3D& from, const EdgeList3D& to, Float epsilon = EPSILON);

    inline bool AreIsomorphic(const EdgeList3D& from, const EdgeList3D& to, Float epsilon = EPSILON) {
        return FindIsomorphism(from, to, epsilon).has_value();
    }

    EdgeList3D
    CreateEdgeList(const std::vector<Vertex3D>& vertices, const std::vector<std::vector<std::size_t>>& faces);
} // namespace compg
