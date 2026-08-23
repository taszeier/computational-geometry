#include "delaunay/DelaunayTriangulator.hpp"

#include "delaunay/TriangleSearchStructure.hpp"
#include "math/Geometry.hpp"
#include <utility>

namespace compg {
    void Flip(
        DoublyConnectedEdgeList::vertex_index v1, DoublyConnectedEdgeList::vertex_index v2,
        DoublyConnectedEdgeList& edgeList
    ) {
        const auto e1 = edgeList.EdgeIndexMap.at(DoublyConnectedEdgeList::edge_type{v1, v2});
        const auto e2 = edgeList.GetTwinIndex(e1);
        const auto e3 = edgeList.GetNextIndex(e1);
        const auto e4 = edgeList.GetPreviousIndex(e2);
        const auto e5 = edgeList.GetPreviousIndex(e1);
        const auto e6 = edgeList.GetNextIndex(e2);

        const auto v3 = edgeList.GetOriginIndex(e5);
        edgeList.Edges.at(e1) = DoublyConnectedEdgeList::EdgeRecord{v3, e2, e4, e3, 0};
        const auto v4 = edgeList.GetOriginIndex(e4);
        edgeList.Edges.at(e2) = DoublyConnectedEdgeList::EdgeRecord{v4, e1, e5, e6, 0};

        edgeList.Edges.at(e3).NextIndex = e1;
        edgeList.Edges.at(e3).PreviousIndex = e4;
        edgeList.Edges.at(e4).NextIndex = e3;
        edgeList.Edges.at(e4).PreviousIndex = e1;
        edgeList.Edges.at(e5).NextIndex = e6;
        edgeList.Edges.at(e5).PreviousIndex = e2;
        edgeList.Edges.at(e6).NextIndex = e2;
        edgeList.Edges.at(e6).PreviousIndex = e5;

        edgeList.EdgeIndexMap.erase(DoublyConnectedEdgeList::edge_type{v1, v2});
        edgeList.EdgeIndexMap.erase(DoublyConnectedEdgeList::edge_type{v2, v1});
        edgeList.EdgeIndexMap[DoublyConnectedEdgeList::edge_type{v3, v4}] = e1;
        edgeList.EdgeIndexMap[DoublyConnectedEdgeList::edge_type{v4, v3}] = e2;

        edgeList.Vertices.at(v1).IncidentEdgeIndex = e6;
        edgeList.Vertices.at(v2).IncidentEdgeIndex = e3;
        edgeList.AreFacesValid_ = false;
    }

    namespace details {

        struct OppositeVertexMap {
            using edge_type = DirectedEdge<VertexIndex>;
            using map_type = std::unordered_map<edge_type, VertexIndex>;

            void UpdateTriangle(VertexIndex v0, VertexIndex v1, VertexIndex v2) {
                Map.insert_or_assign(edge_type{v0, v1}, v2);
                Map.insert_or_assign(edge_type{v1, v2}, v0);
                Map.insert_or_assign(edge_type{v2, v0}, v1);
            }

            void UpdateTriangle(const IndexTriangle& triangle) {
                UpdateTriangle(triangle[0], triangle[1], triangle[2]);
            }

            void OnCenterSplit(const IndexTriangle& triangle, VertexIndex vertex) {
                const auto [v0, v1, v2] = triangle.Indices;
                UpdateTriangle(v0, v1, vertex);
                UpdateTriangle(v1, v2, vertex);
                UpdateTriangle(v2, v0, vertex);
            }

            void OnEdgeSplit(const AdjacentTriangles& triangles, VertexIndex vertex) {
                const auto v0 = triangles.CommonEdge[0];
                const auto v1 = triangles.CommonEdge[1];
                Map.erase(edge_type{v0, v1});
                Map.erase(edge_type{v1, v0});

                UpdateTriangle(v0, triangles.RightVertex, vertex);
                UpdateTriangle(triangles.RightVertex, v1, vertex);
                UpdateTriangle(v1, triangles.LeftVertex, vertex);
                UpdateTriangle(triangles.LeftVertex, v0, vertex);
            }

            void OnEdgeFlip(const AdjacentTriangles& triangles) {
                const auto v0 = triangles.CommonEdge[0];
                const auto v1 = triangles.CommonEdge[1];
                Map.erase(edge_type{v0, v1});
                Map.erase(edge_type{v1, v0});

                UpdateTriangle(triangles.LeftVertex, v0, triangles.RightVertex);
                UpdateTriangle(triangles.RightVertex, v1, triangles.LeftVertex);
            }

            auto At(const edge_type& edge) const {
                return Map.at(edge);
            }

            auto At(VertexIndex v0, VertexIndex v1) const {
                return At(edge_type{v0, v1});
            }

            template <typename... Args>
            auto Erase(Args&&... args) {
                Map.erase(std::forward<Args>(args)...);
            }

            map_type Map;
        };

        std::optional<DirectedEdge<VertexIndex>>
        FindIncidentEdge(const IndexTriangle& triangle, const Vertex2D& vertex, const auto& vertexLocator) {
            for (std::size_t i = 0; i < 3; ++i) {
                const auto v0 = triangle.Indices.at(i);
                const auto v1 = triangle.Indices.at((i + 1) % 3);
                if (v0.Index.index() == 2 && v1.Index.index() == 2) {
                    const auto i0 = std::get<2>(v0.Index);
                    const auto i1 = std::get<2>(v1.Index);
                    if (FindSide(vertexLocator(i0), vertexLocator(i1), vertex) == PointSide::Collinear) {
                        return DirectedEdge<VertexIndex>{v0, v1};
                    }
                }
            }
            return std::nullopt;
        }

        bool IsInsideCircumcircle(
            std::size_t i0, std::size_t i1, std::size_t i2, std::size_t queryIndex, const auto& vertexLocator
        ) {
            const auto v0 = vertexLocator(i0);
            const auto v1 = vertexLocator(i1);
            const auto v2 = vertexLocator(i2);
            const auto query = vertexLocator(queryIndex);

            const auto bisector12 = CreateBisector(v0, v1);
            const auto bisector13 = CreateBisector(v0, v2);
            const auto circumcenter = FindIntersection(bisector12, bisector13);
            COMPG_ASSERT(circumcenter.has_value(), "The vertices are collinear");

            const auto radiusSquared = (v0 - circumcenter.value()).squaredNorm();
            return (circumcenter.value() - query).squaredNorm() < radiusSquared;
        }

        bool IsLegal(
            VertexIndex leftVertex, DirectedEdge<VertexIndex> edge, const auto& vertexLocator,
            const OppositeVertexMap& oppositeVertexMap
        ) {
            // TODO: refactor this mess.
            if (edge == DirectedEdge<VertexIndex>{{Minus2{}}, {Minus1{}}}
                || edge == DirectedEdge<VertexIndex>{{Minus1{}}, {0UZ}}
                || edge == DirectedEdge<VertexIndex>{{0UZ}, {Minus2{}}}) {
                return true;
            }

            const auto rightVertex = oppositeVertexMap.At(edge[1], edge[0]);
            if (leftVertex.Index.index() == 2 && rightVertex.Index.index() == 2 && edge[0].Index.index() == 2
                && edge[1].Index.index() == 2) {
                return !IsInsideCircumcircle(
                    std::get<2>(leftVertex.Index), std::get<2>(edge[0].Index), std::get<2>(edge[1].Index),
                    std::get<2>(rightVertex.Index), vertexLocator
                );
            }

            const auto mkl = std::min(leftVertex, rightVertex);
            const auto mij = std::min(edge[0], edge[1]);
            if (mkl < mij) {
                return true;
            }
            if (mij == VertexIndex{Minus2{}} && mkl == VertexIndex{Minus1{}}) {
                return true;
            }

            // k,l >= 0
            COMPG_ASSERT(leftVertex.Index.index() == 2 && rightVertex.Index.index() == 2, "Oops");
            const auto l = vertexLocator(std::get<2>(leftVertex.Index));
            const auto r = vertexLocator(std::get<2>(rightVertex.Index));
            if (edge[0] < VertexIndex{0UZ}) {
                COMPG_ASSERT(edge[1].Index.index() == 2, "Oops");
                const auto j = vertexLocator(std::get<2>(edge[1].Index));
                const auto angle = Angle360(l - j, r - j);
                return angle >= math::PI;
            } else {
                COMPG_ASSERT(edge[0].Index.index() == 2, "Oops");
                const auto i = vertexLocator(std::get<2>(edge[0].Index));
                const auto angle = Angle360(r - i, l - i);
                return angle >= math::PI;
            }
        }

        class DelaunayTriangulatorState {
        public:
            static DelaunayTriangulatorState Create(const Vertex2D& largestVertex) {
                DoublyConnectedEdgeList triangulation;
                const auto vertexIndex = triangulation.InsertVertex(largestVertex);
                const IndexTriangle rootTriangle{
                    VertexIndex{Minus2{}}, VertexIndex{Minus1{}}, VertexIndex{vertexIndex}
                };

                const TriangleSearchStructure searchStructure(rootTriangle);
                OppositeVertexMap oppositeVertexMap{};
                oppositeVertexMap.UpdateTriangle(rootTriangle);

                return DelaunayTriangulatorState{searchStructure, oppositeVertexMap, triangulation};
            }

            void InsertVertex(const Vertex2D& vertex) {
                auto vertexLocator = [this](std::size_t i) { return Triangulation.GetVertex(i).Vertex; };
                const auto triangle = SearchStructure.Search(vertex, vertexLocator);
                const auto incidentEdge = FindIncidentEdge(triangle, vertex, vertexLocator);
                if (incidentEdge.has_value()) {
                    HandleEdgeSplit(vertex, incidentEdge.value());
                } else {
                    HandleCenterSplit(vertex, triangle);
                }
            }

            auto GetTriangulation() const {
                return Triangulation;
            }

        private:
            DelaunayTriangulatorState(
                TriangleSearchStructure searchStructure, OppositeVertexMap vertexMap,
                DoublyConnectedEdgeList triangulation
            )
                : SearchStructure{std::move(searchStructure)}
                , VertexMap{std::move(vertexMap)}
                , Triangulation{std::move(triangulation)} {}

            void HandleCenterSplit(const Vertex2D& vertex, const IndexTriangle& triangle) {
                const auto vertexIndex = Triangulation.InsertVertex(vertex);
                SearchStructure.OnCenterSplit(triangle, VertexIndex{vertexIndex});
                VertexMap.OnCenterSplit(triangle, VertexIndex{vertexIndex});

                std::ranges::for_each(triangle.Indices, [this, vertexIndex](VertexIndex tvi) {
                    if (tvi.Index.index() == 2) {
                        Triangulation.InsertEdge(vertexIndex, std::get<2>(tvi.Index));
                    }
                });

                LegalizeEdge(VertexIndex{vertexIndex}, DirectedEdge<VertexIndex>{triangle[0], triangle[1]});
                LegalizeEdge(VertexIndex{vertexIndex}, DirectedEdge<VertexIndex>{triangle[1], triangle[2]});
                LegalizeEdge(VertexIndex{vertexIndex}, DirectedEdge<VertexIndex>{triangle[2], triangle[0]});
            }

            void HandleEdgeSplit(const Vertex2D& vertex, const DirectedEdge<VertexIndex>& incidentEdge) {
                const auto leftVertex = VertexMap.At(incidentEdge.At(0), incidentEdge.At(1));
                const auto rightVertex = VertexMap.At(incidentEdge.At(1), incidentEdge.At(0));

                const AdjacentTriangles triangles{incidentEdge, leftVertex, rightVertex};

                COMPG_ASSERT(incidentEdge.At(0).Index.index() == 2, "Oops");
                COMPG_ASSERT(incidentEdge.At(1).Index.index() == 2, "Oops");
                const auto vertexIndex = Triangulation.Split(
                    std::get<2>(incidentEdge.At(0).Index), std::get<2>(incidentEdge.At(1).Index), vertex
                );
                if (leftVertex.Index.index() == 2) {
                    Triangulation.InsertEdge(std::get<2>(leftVertex.Index), vertexIndex);
                }
                if (rightVertex.Index.index() == 2) {
                    Triangulation.InsertEdge(std::get<2>(rightVertex.Index), vertexIndex);
                }
                SearchStructure.OnEdgeSplit(triangles, VertexIndex{vertexIndex});
                VertexMap.OnEdgeSplit(triangles, VertexIndex{vertexIndex});

                LegalizeEdge(VertexIndex{vertexIndex}, DirectedEdge<VertexIndex>{incidentEdge.At(0), rightVertex});
                LegalizeEdge(VertexIndex{vertexIndex}, DirectedEdge<VertexIndex>{rightVertex, incidentEdge.At(1)});
                LegalizeEdge(VertexIndex{vertexIndex}, DirectedEdge<VertexIndex>{incidentEdge.At(1), leftVertex});
                LegalizeEdge(VertexIndex{vertexIndex}, DirectedEdge<VertexIndex>{leftVertex, incidentEdge.At(0)});
            }

            void LegalizeEdge(VertexIndex leftVertex, const DirectedEdge<VertexIndex>& edge) {
                auto vertexLocator = [this](std::size_t i) { return Triangulation.GetVertex(i).Vertex; };
                if (!IsLegal(leftVertex, edge, vertexLocator, VertexMap)) {
                    const auto rightVertex = VertexMap.At(edge[1], edge[0]);
                    Flip(AdjacentTriangles{edge, leftVertex, rightVertex});
                    LegalizeEdge(leftVertex, DirectedEdge<VertexIndex>{edge[0], rightVertex});
                    LegalizeEdge(leftVertex, DirectedEdge<VertexIndex>{rightVertex, edge[1]});
                }
            }

            void Flip(const AdjacentTriangles& triangles) {
                if (std::min(triangles.CommonEdge[0], triangles.CommonEdge[1]) < VertexIndex{0UZ}) {
                    if (!(std::min(triangles.LeftVertex, triangles.RightVertex) < VertexIndex{0UZ})) {
                        const auto li = std::get<2>(triangles.LeftVertex.Index);
                        const auto ri = std::get<2>(triangles.RightVertex.Index);
                        Triangulation.InsertEdge(li, ri);
                    }
                } else {
                    compg::Flip(
                        std::get<2>(triangles.CommonEdge[0].Index), std::get<2>(triangles.CommonEdge[1].Index),
                        Triangulation
                    );
                }

                SearchStructure.OnEdgeFlip(triangles);
                VertexMap.OnEdgeFlip(triangles);
            }

        private:
            TriangleSearchStructure SearchStructure;
            OppositeVertexMap VertexMap;
            DoublyConnectedEdgeList Triangulation;
        };
    } // namespace details

    DoublyConnectedEdgeList
    DelaunayTriangulator::Triangulate(const std::vector<Vertex2D>& vertices, std::size_t seed) const {
        if (vertices.empty()) {
            return DoublyConnectedEdgeList{};
        }

        auto permutation{vertices};
        RandomPermutation(permutation, seed);

        const auto maxIt = std::ranges::max_element(permutation, DelaunayLess{});
        auto state = details::DelaunayTriangulatorState::Create(*maxIt);
        permutation.erase(maxIt);

        std::ranges::for_each(permutation, [&state](const auto& vertex) { state.InsertVertex(vertex); });
        return state.GetTriangulation();
    }
} // namespace compg
