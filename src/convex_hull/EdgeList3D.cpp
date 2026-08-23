#include "convex_hull/EdgeList3D.hpp"
#include "data_structures/VertexFinder.hpp"

namespace compg {
    std::optional<EdgeList3D::Isomorphism>
    FindIsomorphism(const EdgeList3D& from, const EdgeList3D& to, Float epsilon) {
        if (from.NumVertices() != to.NumVertices() || from.NumEdges() != to.NumEdges()) {
            return std::nullopt;
        }
        if (from.NumVertices() == 0) {
            return EdgeList3D::Isomorphism{};
        }

        const auto vertices = std::views::iota(0UZ, to.NumVertices())
                              | std::views::transform([&to](const auto i) { return to.GetVertex(i).Vertex; })
                              | std::ranges::to<std::vector>();
        VertexFinder<3UZ> finder{vertices};

        std::unordered_map<EdgeList3D::vertex_index, EdgeList3D::vertex_index> vertexIsomorphism;
        vertexIsomorphism.reserve(from.NumVertices());
        for (const auto vertexIndex : from.GetVertexIndices()) {
            const auto vertex = from.GetVertex(vertexIndex).Vertex;
            const auto is = finder.Find(vertex, epsilon);
            if (is.size() != 1) {
                return std::nullopt;
            }
            vertexIsomorphism[vertexIndex] = is.at(0);
        }

        std::unordered_map<EdgeList3D::edge_index, EdgeList3D::edge_index> edgeIsomorphism;
        edgeIsomorphism.reserve(from.NumEdges());
        for (const auto& [edge, edgeIndex] : from.EdgeIndexMap) {
            const EdgeList3D::edge_type edgeMapped{vertexIsomorphism.at(edge.At(0)), vertexIsomorphism.at(edge.At(1))};
            if (!to.EdgeIndexMap.contains(edgeMapped)) {
                return std::nullopt;
            }
            edgeIsomorphism[edgeIndex] = to.EdgeIndexMap.at(edgeMapped);
        }

        return EdgeList3D::Isomorphism{vertexIsomorphism, edgeIsomorphism};
    }

    EdgeList3D
    CreateEdgeList(const std::vector<Vertex3D>& vertices, const std::vector<std::vector<std::size_t>>& faces) {
        EdgeList3D edgeList;
        const auto indices
            = std::views::transform(vertices, [&edgeList](const auto& v) { return edgeList.AddVertex(v); })
              | std::ranges::to<std::vector>();

        for (const auto& face : faces) {
            COMPG_ASSERT(face.size() >= 3, "Expected at least three vertices on the face");

            const auto v0 = indices.at(face.at(0));
            const auto v1 = indices.at(face.at(1));
            const auto e0 = edgeList.InsertEdge(v0, v1);
            const auto faceIndex = edgeList.AddFace(e0);
            edgeList.Edges.at(e0).FaceIndex = faceIndex;

            auto prevEdge = e0;
            for (std::size_t i = 1; i < face.size(); ++i) {
                const auto j = (i + 1) % face.size();
                const auto e = edgeList.InsertEdge(indices.at(face.at(i)), indices.at(face.at(j)));
                edgeList.Edges.at(e).FaceIndex = faceIndex;

                edgeList.MakeAdjacent(prevEdge, e);
                prevEdge = e;
            }

            edgeList.MakeAdjacent(prevEdge, e0);
        }

        return edgeList;
    }
} // namespace compg