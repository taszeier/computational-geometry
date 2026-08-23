#include "convex_hull/ConvexHull3DCalculator.hpp"
#include "data_structures/graph/BipartiteGraph.hpp"
#include "math/Geometry.hpp"

namespace compg {
    using graph_type = BipartiteGraph<std::size_t, EdgeList3D::face_index>;

    namespace details {
        auto CreateHalfPlane(const Vertex3D& v0, const Vertex3D& v1, const Vertex3D& v2) {
            const auto plane = FindHyperplane(v0, v1, v2);
            return HalfPlane<Hyperplane3D>{plane};
        }

        /**
         * @brief Find the indices of four vertices that define a tetrahedron.
         */
        auto FindTetrahedronIndices(const std::vector<Vertex3D>& vertices, Float epsilon) {
            COMPG_ASSERT(!vertices.empty(), "Expected a non-empty vector");
            constexpr std::size_t i0{};
            const auto it1
                = std::find_if(std::next(vertices.begin()), vertices.end(), [&vertices, i0, epsilon](const auto& v) {
                      return !AreEqual(v, vertices.at(i0), epsilon);
                  });

            COMPG_ASSERT(it1 != vertices.end(), "Expected a vector with at least two non-equivalent vertices");
            const std::size_t i1 = std::distance(vertices.begin(), it1);

            const Line3D line{vertices.at(i0), vertices.at(i1)};
            const auto it2 = std::find_if(std::next(it1), vertices.end(), [&line, epsilon](const auto& v) {
                return !Contains(line, v, epsilon);
            });
            COMPG_ASSERT(it2 != vertices.end(), "Expected non-collinear vertices");
            const std::size_t i2 = std::distance(vertices.begin(), it2);

            const auto halfPlane = CreateHalfPlane(vertices.at(i0), vertices.at(i1), vertices.at(i2));
            const auto it3 = std::find_if(std::next(it2), vertices.end(), [&halfPlane, epsilon](const auto& v) {
                return FindSide<3UZ>(halfPlane, v, epsilon) != PointSide::Collinear;
            });
            // TODO: call 2D convex algorithm instead of throwing
            COMPG_ASSERT(it3 != vertices.end(), "Expected non-coplanar vertices");
            const std::size_t i3 = std::distance(vertices.begin(), it3);

            return std::array{i0, i1, i2, i3};
        }

        /**
         * @brief Create a tetrahedron from four non-coplanar vertices where the fourth vertex is on the negative side
         * of the plane defined by the first three vertices.
         */
        EdgeList3D
        CreateTetrahedronImpl(const Vertex3D& v0, const Vertex3D& v1, const Vertex3D& v2, const Vertex3D& v3) {
            EdgeList3D edgeList;
            const auto i0 = edgeList.AddVertex(v0);
            const auto i1 = edgeList.AddVertex(v1);
            const auto i2 = edgeList.AddVertex(v2);
            const auto i3 = edgeList.AddVertex(v3);
            edgeList.InsertEdge(i0, i1);
            edgeList.InsertEdge(i1, i2);
            edgeList.InsertEdge(i2, i0);
            edgeList.InsertEdge(i3, i0);
            edgeList.InsertEdge(i3, i1);
            edgeList.InsertEdge(i3, i2);

            const std::array<std::array<EdgeList3D::vertex_index, 3>, 4> faces{
                {{i0, i1, i2}, {i1, i0, i3}, {i0, i2, i3}, {i2, i1, i3}}
            };
            for (const auto& face : faces) {
                const auto [i, j, k] = face;
                const auto e0 = edgeList.GetEdgeIndex(i, j);
                const auto e1 = edgeList.GetEdgeIndex(j, k);
                const auto e2 = edgeList.GetEdgeIndex(k, i);

                const auto faceIndex = edgeList.AddFace(e0);
                edgeList.MakeAdjacent(e0, e1);
                edgeList.MakeAdjacent(e1, e2);
                edgeList.MakeAdjacent(e2, e0);

                edgeList.Edges.at(e0).FaceIndex = faceIndex;
                edgeList.Edges.at(e1).FaceIndex = faceIndex;
                edgeList.Edges.at(e2).FaceIndex = faceIndex;
            }

            return edgeList;
        }

        /**
         * @brief Create a tetrahedron from four non-coplanar vertices.
         */
        EdgeList3D CreateTetrahedron(
            const Vertex3D& v0, const Vertex3D& v1, const Vertex3D& v2, const Vertex3D& v3, Float epsilon = EPSILON
        ) {
            const auto halfPlane = CreateHalfPlane(v0, v1, v2);
            if (FindSide(halfPlane, v3, epsilon) == PointSide::Positive) {
                return CreateTetrahedronImpl(v0, v2, v1, v3);
            }
            return CreateTetrahedronImpl(v0, v1, v2, v3);
        }

        graph_type CreateConflictGraph(
            const std::vector<Vertex3D>& vertices, const std::vector<std::size_t>& indices, const EdgeList3D& edgeList,
            Float epsilon = EPSILON
        ) {
            graph_type graph;

            for (const auto& vertexIndex : indices) {
                graph.InsertLeftNode(vertexIndex);
            }
            for (std::size_t faceIndex = 0; faceIndex < edgeList.Faces.size(); ++faceIndex) {
                graph.InsertRightNode(faceIndex);
            }

            for (std::size_t faceIndex = 0; faceIndex < edgeList.Faces.size(); ++faceIndex) {
                const auto e0 = edgeList.GetFace(faceIndex).IncidentEdge;
                const auto e1 = edgeList.GetNextIndex(e0);
                const auto e2 = edgeList.GetNextIndex(e1);
                const auto halfPlane = CreateHalfPlane(
                    edgeList.GetOrigin(e0).Vertex, edgeList.GetOrigin(e1).Vertex, edgeList.GetOrigin(e2).Vertex
                );
                for (const auto& vertexIndex : indices) {
                    const auto side = FindSide(halfPlane, vertices.at(vertexIndex), epsilon);
                    if (side == PointSide::Positive) {
                        graph.InsertEdge(vertexIndex, faceIndex);
                    }
                }
            }

            return graph;
        }

        /**
         * @brief Collect the edges that are incident to a conflicting face and a non-conflicting face (horizon edges)
         * and the edges that are incident to two conflicting faces (interior edges).
         */
        auto CollectVisibleEdges(const auto& conflictingFaces, const EdgeList3D& edgeList) {
            std::vector<EdgeList3D::edge_index> horizonEdges;
            std::unordered_set<EdgeList3D::edge_index> interiorEdges;
            for (const EdgeList3D::face_index faceIndex : conflictingFaces) {
                const auto startEdge = edgeList.GetFace(faceIndex).IncidentEdge;
                auto currentEdge = startEdge;
                do {
                    const auto twinEdge = edgeList.GetTwinIndex(currentEdge);
                    const auto adjacentFace = edgeList.GetFaceIndex(twinEdge);
                    if (!conflictingFaces.contains(adjacentFace)) {
                        horizonEdges.push_back(currentEdge);
                    } else if (!interiorEdges.contains(edgeList.GetTwinIndex(currentEdge))) {
                        interiorEdges.insert(currentEdge);
                    }

                    currentEdge = edgeList.GetNextIndex(currentEdge);
                } while (currentEdge != startEdge);
            }

            return std::tuple{horizonEdges, interiorEdges};
        }

        class ConvexHull3DState {
        public:
            ConvexHull3DState(
                const std::vector<Vertex3D>& vertices, const std::vector<std::size_t>& indices,
                const EdgeList3D& edgeList, const graph_type& conflictGraph, Float epsilon
            )
                : Vertices{vertices}
                , Indices{indices}
                , EdgeList{edgeList}
                , ConflictGraph{conflictGraph}
                , Epsilon{epsilon} {}

            /**
             * @brief Initialize the state of the convex hull algorithm.
             * @param vertices A vector of vertices.
             * @param epsilon The error threshold.
             * @param seed The seed used to improve average running time.
             */
            static ConvexHull3DState Create(const std::vector<Vertex3D>& vertices, Float epsilon, std::size_t seed) {
                const auto tetrahedronIndices = details::FindTetrahedronIndices(vertices, epsilon);
                auto edgeList = details::CreateTetrahedron(
                    vertices.at(tetrahedronIndices.at(0)), vertices.at(tetrahedronIndices.at(1)),
                    vertices.at(tetrahedronIndices.at(2)), vertices.at(tetrahedronIndices.at(3))
                );
                auto indices = std::views::iota(0UZ, vertices.size())
                               | std::views::filter([&tetrahedronIndices](const auto i) {
                                     return std::ranges::find(tetrahedronIndices, i) == tetrahedronIndices.end();
                                 })
                               | std::ranges::to<std::vector>();
                RandomPermutation(indices, seed);
                auto conflictGraph = details::CreateConflictGraph(vertices, indices, edgeList, epsilon);

                return ConvexHull3DState{vertices, indices, edgeList, conflictGraph, epsilon};
            }

            /**
             * @brief Insert the vertices to create the convex hull.
             */
            void Run() {
                for (const auto i : Indices) {
                    HandleInsertion(i);
                }
            }

            /**
             * @brief Insert a vertex into the convex hull.
             * @param i The index of the vertex.
             */
            void HandleInsertion(std::size_t i) {
                const auto conflictingFaces = ConflictGraph.GetRightAdjacentNodes(i);
                if (!conflictingFaces.empty()) {
                    const auto [horizonEdges, interiorEdges] = details::CollectVisibleEdges(conflictingFaces, EdgeList);
                    std::unordered_set<EdgeList3D::vertex_index> horizonVertices;
                    for (const auto edgeIndex : horizonEdges) {
                        horizonVertices.insert(EdgeList.GetOriginIndex(edgeIndex));
                    }

                    const EdgeList3D::vertex_index vertexIndex = EdgeList.AddVertex(Vertices.at(i));

                    std::vector<EdgeList3D::edge_index> newEdges;
                    newEdges.reserve(horizonEdges.size());
                    for (const auto edgeIndex : horizonEdges) {
                        const auto originIndex = EdgeList.GetOriginIndex(edgeIndex);
                        const auto newEdgeIndex = EdgeList.InsertEdge(originIndex, vertexIndex);
                        newEdges.push_back(newEdgeIndex);
                    }

                    for (const auto edgeIndex : horizonEdges) {
                        const auto originIndex = EdgeList.GetOriginIndex(edgeIndex);
                        const auto twinIndex = EdgeList.GetTwinIndex(edgeIndex);
                        const auto destinationIndex = EdgeList.GetOriginIndex(twinIndex);

                        const auto prev = EdgeList.GetEdgeIndex(vertexIndex, originIndex);
                        const auto next = EdgeList.GetEdgeIndex(destinationIndex, vertexIndex);
                        EdgeList.Vertices.at(vertexIndex).IncidentEdge = prev;

                        const auto v0 = EdgeList.GetOriginIndex(EdgeList.GetPreviousIndex(twinIndex));
                        const auto halfPlane = details::CreateHalfPlane(
                            EdgeList.GetVertex(v0).Vertex, EdgeList.GetVertex(destinationIndex).Vertex,
                            EdgeList.GetVertex(originIndex).Vertex
                        );

                        if (FindSide(halfPlane, Vertices.at(i), Epsilon) == PointSide::Collinear) {
                            HandleFaceCoplanar(edgeIndex, next, prev);
                        } else {
                            HandleFaceNegative(edgeIndex, next, prev);
                        }
                    }

                    // This step is not in the book for some reason.
                    const auto incidentEdge = RemoveUnnecessaryEdges(newEdges);
                    COMPG_ASSERT(incidentEdge.has_value(), "Expected at least one non-coplanar invisible face.");
                    EdgeList.Vertices.at(vertexIndex).IncidentEdge = incidentEdge;

                    HandleErasure(i, conflictingFaces, horizonVertices, interiorEdges);
                }
            }

            /**
             * @brief Handle the insertion of a vertex when it is coplanar to a face whose edge is on the horizon.
             * @param horizonEdgeIndex The index of the half edge on the horizon that is incident to a conflicting face.
             * @param next The index of the half edge after the horizon edge.
             * @param prev The index of the half edge before the horizon edge.
             */
            void HandleFaceCoplanar(
                EdgeList3D::edge_index horizonEdgeIndex, EdgeList3D::edge_index next, EdgeList3D::edge_index prev
            ) {
                const auto originIndex = EdgeList.GetOriginIndex(horizonEdgeIndex);
                const auto twinIndex = EdgeList.GetTwinIndex(horizonEdgeIndex);
                const auto destinationIndex = EdgeList.GetOriginIndex(twinIndex);
                const auto e1 = EdgeList.GetPreviousIndex(twinIndex);
                const auto e3 = EdgeList.GetNextIndex(twinIndex);

                EdgeList.MakeAdjacent(next, prev);
                EdgeList.MakeAdjacent(e1, next);
                EdgeList.MakeAdjacent(prev, e3);

                const auto x = EdgeList.GetFaceIndex(twinIndex);
                EdgeList.Edges.at(next).FaceIndex = x;
                EdgeList.Edges.at(prev).FaceIndex = x;
                EdgeList.Vertices.at(originIndex).IncidentEdge = e3;
                EdgeList.Vertices.at(destinationIndex).IncidentEdge = next;
                EdgeList.Faces.at(x).IncidentEdge = e1;

                EdgeList.EraseEdge(horizonEdgeIndex);
            }

            /**
             * @brief Handle the insertion of a vertex when it is on the negative side of a face whose edge is on the
             * horizon.
             * @param horizonEdgeIndex The index of the half edge on the horizon that is incident to a conflicting face.
             * @param next The index of the half edge after the horizon edge.
             * @param prev The index of the half edge before the horizon edge.
             */
            void HandleFaceNegative(
                EdgeList3D::edge_index horizonEdgeIndex, EdgeList3D::edge_index next, EdgeList3D::edge_index prev
            ) {
                const auto originIndex = EdgeList.GetOriginIndex(horizonEdgeIndex);
                const auto twinIndex = EdgeList.GetTwinIndex(horizonEdgeIndex);
                const auto destinationIndex = EdgeList.GetOriginIndex(twinIndex);

                EdgeList.Vertices.at(originIndex).IncidentEdge = horizonEdgeIndex;
                EdgeList.Vertices.at(destinationIndex).IncidentEdge = next;

                const auto visibleFaceIndex = EdgeList.GetFaceIndex(horizonEdgeIndex);
                const auto invisibleFaceIndex = EdgeList.GetFaceIndex(twinIndex);

                const auto faceIndex = EdgeList.AddFace(horizonEdgeIndex);

                EdgeList.MakeAdjacent(next, prev);
                EdgeList.MakeAdjacent(horizonEdgeIndex, next);
                EdgeList.MakeAdjacent(prev, horizonEdgeIndex);

                EdgeList.Edges.at(horizonEdgeIndex).FaceIndex = faceIndex;
                EdgeList.Edges.at(next).FaceIndex = faceIndex;
                EdgeList.Edges.at(prev).FaceIndex = faceIndex;

                const auto vertexIndex = EdgeList.GetOriginIndex(prev);

                const auto newFaceHalfPlane = details::CreateHalfPlane(
                    EdgeList.GetVertex(originIndex).Vertex, EdgeList.GetVertex(destinationIndex).Vertex,
                    EdgeList.GetVertex(vertexIndex).Vertex
                );
                ConflictGraph.InsertRightNode(faceIndex);
                for (const auto f : std::array{visibleFaceIndex, invisibleFaceIndex}) {
                    for (const auto j : ConflictGraph.GetLeftAdjacentNodes(f)) {
                        const auto side = FindSide<3>(newFaceHalfPlane, Vertices.at(j), Epsilon);
                        if (side == PointSide::Positive) {
                            ConflictGraph.InsertEdge(j, faceIndex);
                        }
                    }
                }
            }

            /**
             * @brief Erase vertices, edges and faces that are no longer on the surface of the convex hull.
             * @param vertexIndex The index of the inserted vertex.
             * @param conflictingFaces The face indices that are conflicting.
             * @param horizonVertices The vertex indices on the horizon.
             * @param interiorEdges The edge indices that are visible from the inserted vertex.
             */
            void HandleErasure(
                std::size_t vertexIndex, const auto& conflictingFaces, const auto& horizonVertices,
                const auto& interiorEdges
            ) {
                for (const auto edgeIndex : interiorEdges) {
                    const auto originIndex = EdgeList.GetOriginIndex(edgeIndex);
                    if (!horizonVertices.contains(originIndex)) {
                        EdgeList.EraseVertex(originIndex);
                    }
                    const auto destinationIndex = EdgeList.GetDestinationIndex(edgeIndex);
                    if (!horizonVertices.contains(destinationIndex)) {
                        EdgeList.EraseVertex(destinationIndex);
                    }
                    EdgeList.EraseEdge(edgeIndex);
                }
                for (const auto faceIndex : conflictingFaces) {
                    EdgeList.EraseFace(faceIndex);
                }

                ConflictGraph.EraseLeftNode(vertexIndex);
                for (const auto faceIndex : conflictingFaces) {
                    ConflictGraph.EraseRightNode(faceIndex);
                }
            }

            /**
             * @brief Remove newly inserted edges that are incident to the same face.
             * @param newEdges The indices of the new edges connecting the horizon vertices to the newly inserted
             * vertex.
             */
            std::optional<EdgeList3D::edge_index> RemoveUnnecessaryEdges(const auto& newEdges) {
                std::optional<EdgeList3D::edge_index> incidentEdge;
                for (const auto edgeIndex : newEdges) {
                    const auto faceIndex = EdgeList.GetFaceIndex(edgeIndex);
                    const auto twinIndex = EdgeList.GetTwinIndex(edgeIndex);
                    const auto twinFaceIndex = EdgeList.GetFaceIndex(twinIndex);
                    if (faceIndex == twinFaceIndex) {
                        const auto next = EdgeList.GetNextIndex(edgeIndex);
                        const auto prev = EdgeList.GetPreviousIndex(twinIndex);
                        EdgeList.MakeAdjacent(prev, next);
                        EdgeList.Faces.at(faceIndex).IncidentEdge = next;

                        EdgeList.EraseVertex(EdgeList.GetOriginIndex(edgeIndex));
                        EdgeList.EraseEdge(edgeIndex);
                    } else {
                        incidentEdge = twinIndex;
                    }
                }
                return incidentEdge;
            }

            auto GetResult() const {
                return ConvexHull3D{EdgeList};
            }

        private:
            const std::vector<Vertex3D>& Vertices;
            std::vector<std::size_t> Indices;
            EdgeList3D EdgeList;
            graph_type ConflictGraph;
            Float Epsilon;
        };

    } // namespace details

    ConvexHull3D ConvexHull3DCalculator::FindConvexHull(const std::vector<Vertex3D>& vertices) const {
        auto state = details::ConvexHull3DState::Create(vertices, Epsilon, Seed);
        state.Run();
        return state.GetResult();
    }
} // namespace compg