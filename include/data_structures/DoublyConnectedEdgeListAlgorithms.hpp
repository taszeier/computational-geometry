#pragma once

#include <stack>

#include "data_structures/DoublyConnectedEdgeList.hpp"

#include <unordered_set>

namespace compg {
    /**
     * @param edgeList A doubly connected edge list.
     * @param edgeIndex An edge index in the edge list to start the walk.
     * @param function The object called on every half edge on the boundary.
     */
    template <typename FunctionType>
    void WalkBoundary(
        const DoublyConnectedEdgeList& edgeList, DoublyConnectedEdgeList::edge_index edgeIndex, FunctionType&& function
    ) {
        COMPG_ASSERT(
            edgeIndex < edgeList.NumEdges(), CreateOutOfRangeMessage("edgeIndex", edgeIndex, edgeList.NumEdges())
        );

        auto currentEdgeIndex = edgeIndex;
        do {
            function(currentEdgeIndex);
            currentEdgeIndex = edgeList.GetNextIndex(currentEdgeIndex);
        } while (currentEdgeIndex != edgeIndex);
    }

    /**
     * @param edgeList A doubly connected edge list.
     * @param vertexIndex A vertex index in the edge list to start the walk.
     * @param function The object called on every half edge whose origin is the vertex index.
     */
    template <typename FunctionType>
    void WalkOutgoingEdges(
        const DoublyConnectedEdgeList& edgeList, DoublyConnectedEdgeList::vertex_index vertexIndex,
        FunctionType&& function
    ) {
        COMPG_ASSERT(
            vertexIndex < edgeList.NumVertices(),
            CreateOutOfRangeMessage("vertexIndex", vertexIndex, edgeList.NumVertices())
        );

        if (const auto& [vertex, incidentEdgeIndex] = edgeList.GetVertex(vertexIndex); incidentEdgeIndex.has_value()) {
            const auto startEdgeIndex = incidentEdgeIndex.value();
            auto currentEdgeIndex = startEdgeIndex;

            do {
                function(currentEdgeIndex);
                currentEdgeIndex = edgeList.GetNextIndex(edgeList.GetTwinIndex(currentEdgeIndex));
            } while (currentEdgeIndex != startEdgeIndex);
        }
    }

    /**
     * @brief Iterate over the edges of a doubly connected edge list.
     * @details The function is called on either a half edge or its twin but never on both.
     * @param edgeList A doubly connected edge list.
     * @param function The object called on a half edge of every edge.
     */
    template <typename FunctionType>
    void WalkUndirectedEdges(const DoublyConnectedEdgeList& edgeList, FunctionType&& function) {
        std::unordered_set<DoublyConnectedEdgeList::edge_index> visited;
        std::ranges::for_each(
            std::views::iota(0UZ, edgeList.NumEdges()), [&visited, &function, &edgeList](auto edgeIndex) {
                if (!visited.contains(edgeIndex)) {
                    function(edgeIndex);
                    visited.insert(edgeIndex);
                    visited.insert(edgeList.GetTwinIndex(edgeIndex));
                }
            }
        );
    }

    /**
     * @brief Perform depth first search on the bounded faces of a doubly connected edge list.
     * @param edgeList A doubly connected edge list.
     * @param function The object called on the bounded faces of the edge list.
     * @param startIndex A face index in the edge list to start the search.
     */
    template <typename FunctionType>
    void DepthFirstSearchBoundedFaces(
        const DoublyConnectedEdgeList& edgeList, FunctionType&& function, DoublyConnectedEdgeList::face_index startIndex
    ) {
        COMPG_ASSERT(edgeList.IsFaceBounded(startIndex), "Cannot start the search at the unbounded face");

        std::stack<DoublyConnectedEdgeList::face_index> stack;
        stack.push(startIndex);
        std::unordered_set<DoublyConnectedEdgeList::face_index> visited;

        while (!stack.empty()) {
            const auto faceIndex = stack.top();
            stack.pop();

            auto addFaceOnOppositeSide = [&visited, &stack, &edgeList](auto edgeIndex) {
                const auto faceIndex = edgeList.GetFaceIndex(edgeList.GetTwinIndex(edgeIndex));
                if (!visited.contains(faceIndex) && edgeList.IsFaceBounded(faceIndex)) {
                    stack.push(faceIndex);
                }
            };

            if (!visited.contains(faceIndex)) {
                function(faceIndex);

                visited.insert(faceIndex);
                const auto face = edgeList.GetFace(faceIndex);
                COMPG_ASSERT(face.OuterComponent.has_value(), "Expected the face to be bounded");
                WalkBoundary(edgeList, face.OuterComponent.value(), addFaceOnOppositeSide);
                for (const auto edgeIndex : face.InnerComponents) {
                    WalkBoundary(edgeList, edgeIndex, addFaceOnOppositeSide);
                }
            }
        }
    }

    /**
     * @brief Find a half edge on each boundary in the doubly connected edge list.
     * @param edgeList A doubly connected edge list.
     * @return A vector of half edge indices, one for each boundary.
     */
    std::vector<DoublyConnectedEdgeList::edge_index> FindBoundaries(const DoublyConnectedEdgeList& edgeList);

    /**
     * @brief Find the vertices on a boundary in the doubly connected edge list.
     * @param edgeList A doubly connected edge list.
     * @param edgeIndex An edge index on a boundary in the edge list.
     * @return A vector of vertex indices on the boundary.
     */
    std::vector<DoublyConnectedEdgeList::vertex_index>
    FindBoundaryVertices(const DoublyConnectedEdgeList& edgeList, DoublyConnectedEdgeList::edge_index edgeIndex);

    /**
     * @brief Find all half edges on each boundary of the doubly connected edge list.
     * @param edgeList A doubly connected edge list.
     * @return A vector of sets of edge indices. Each set of indices corresponds to a boundary.
     */
    std::vector<std::unordered_set<DoublyConnectedEdgeList::edge_index>>
    FindAllBoundaryEdges(const DoublyConnectedEdgeList& edgeList);

    /**
     * @param edgeList A doubly connected edge list.
     * @param edgeIndex An edge index on the boundary to walk.
     * @return The edge index whose origin is the left-most vertex on the boundary.
     */
    DoublyConnectedEdgeList::edge_index
    FindLeftMostEdge(const DoublyConnectedEdgeList& edgeList, DoublyConnectedEdgeList::edge_index edgeIndex);

    /**
     * @param edgeList A doubly connected edge list.
     * @param edgeIndex An edge index on a boundary.
     * @return Whether it is an outer boundary.
     */
    bool IsOuterBoundary(const DoublyConnectedEdgeList& edgeList, DoublyConnectedEdgeList::edge_index edgeIndex);

    /**
     * @param edgeList A doubly connected edge list.
     * @param vertexIndex A vertex index in the edge list.
     * @return A vector of edge indices whose origin is the vertex.
     */
    std::vector<DoublyConnectedEdgeList::edge_index>
    FindOutgoingEdges(const DoublyConnectedEdgeList& edgeList, DoublyConnectedEdgeList::vertex_index vertexIndex);

    /**
     * @brief Find the index of the half edge that would be the successor of the half edge from an origin vertex to a
     * destination vertex.
     * @param edgeList A doubly connected edge list.
     * @param origin The origin of the half edge.
     * @param destination The vertex index of the destination of the half edge.
     * @return The index of the next half edge if the destination vertex has an incident edge, otherwise std::nullopt.
     */
    std::optional<DoublyConnectedEdgeList::edge_index> FindCyclicNext(
        const DoublyConnectedEdgeList& edgeList, const Vertex2D& origin,
        DoublyConnectedEdgeList::vertex_index destination
    );

    inline std::optional<DoublyConnectedEdgeList::edge_index> FindCyclicNext(
        const DoublyConnectedEdgeList& edgeList, DoublyConnectedEdgeList::vertex_index origin,
        DoublyConnectedEdgeList::vertex_index destination
    ) {
        return FindCyclicNext(edgeList, edgeList.GetVertex(origin).Vertex, destination);
    }

    /**
     * @brief Contains the union edge list and the sets of edge indices that are from the left-hand side and right-hand
     * side edge lists, respectively.
     */
    struct UnsafeUnionResult {
        DoublyConnectedEdgeList Union;
        std::unordered_set<DoublyConnectedEdgeList::edge_index> LeftEdges;
        std::unordered_set<DoublyConnectedEdgeList::edge_index> RightEdges;
    };

    /**
     * @brief Take the union of the two doubly connected edge lists.
     * @details The operation is unsafe as it can create edges that cross each other and the next/previous may not be
     * set properly. The two edge lists can contain duplicate edges, which are not duplicated in the output.
     * @param lhs A doubly connected edge list.
     * @param rhs A doubly connected edge list.
     * @return An UnsafeUnionResult.
     */
    UnsafeUnionResult UnsafeUnion(const DoublyConnectedEdgeList& lhs, const DoublyConnectedEdgeList& rhs);

    /**
     * @brief Find the faces induced by the edges and update the edge list accordingly.
     * @details Resets the faces of the edge list and sets the incident face pointer for each half edge record.
     * @param edgeList The edge list to update.
     */
    void UpdateFaces(DoublyConnectedEdgeList& edgeList);

    /**
     * @brief Find the faces induced by the edges and update the edge list accordingly.
     * @details Resets the faces of the edge list and sets the incident face pointer for each half edge record.
     * @param edgeList The edge list to update.
     * @param leftNeighborEdgeMap A map that maps the left-most vertex of each outer boundary to the half edge that is
     * immediately to the left of it.
     */
    void UpdateFaces(
        const std::unordered_map<Vertex2D, DoublyConnectedEdgeList::edge_index>& leftNeighborEdgeMap,
        DoublyConnectedEdgeList& edgeList
    );

    std::unordered_map<std::size_t, DoublyConnectedEdgeList::edge_index>
    InsertEdges(const DoublyConnectedEdgeList& edgeList, std::vector<LineSegment2D>& segments);

    /**
     * @brief Find the isomorphism between two doubly connected edge lists.
     * @param from A doubly connected edge list whose vertices are the domain of the isomorphism.
     * @param to A doubly connected edge list whose vertices are the range of the isomorphism.
     * @param epsilon An error threshold used to match vertices between the edge lists.
     * @return The isomorphism if it exists, otherwise std::nullopt.
     */
    std::optional<DoublyConnectedEdgeList::Isomorphism>
    FindIsomorphism(const DoublyConnectedEdgeList& from, const DoublyConnectedEdgeList& to, Float epsilon = EPSILON);
    inline bool
    AreIsomorphic(const DoublyConnectedEdgeList& lhs, const DoublyConnectedEdgeList& rhs, Float epsilon = EPSILON) {
        return FindIsomorphism(lhs, rhs, epsilon).has_value();
    }

    /**
     * @brief Remove isolated vertices, combine vertices that are close and merge adjacent edges if their angle is 180
     * degrees.
     * @param edgeList A doubly connected edge list.
     * @param epsilon An error threshold.
     * @return The optimized doubly connected edge list.
     */
    DoublyConnectedEdgeList Optimize(const DoublyConnectedEdgeList& edgeList, Float epsilon = EPSILON);

    /**
     * @brief The result of collapsing the vertices of a doubly connected edge list.
     * @details Contains the resulting edge list and a map that assigns each vertex index in the input edge list to the
     * vertex index in the output edge list that it corresponds to.
     */
    struct CollapseVerticesResult {
        DoublyConnectedEdgeList EdgeList;
        std::unordered_map<DoublyConnectedEdgeList::vertex_index, DoublyConnectedEdgeList::vertex_index> VertexMap;
    };

    /**
     * @brief  Combine vertices that are close to each other.
     * @param edgeList A doubly connected edge list.
     * @param epsilon An error threshold used to find duplicate vertices.
     * @return The collapsed vertices result.
     */
    CollapseVerticesResult CollapseVertices(const DoublyConnectedEdgeList& edgeList, Float epsilon = EPSILON);

    /**
     * @brief Find the index of the adjacent edge with the same angle.
     * @details An error is thrown if there are multiple adjacent half edges with the same angle as the input half edge.
     * @param edgeList A doubly connected edge list.
     * @param edgeIndex An edge index in the edge list.
     * @param epsilon An error threshold used to find half edges with the same angle.
     * @return The index of the edge that is adjacent to the input edge index if it exists, otherwise std::nullopt.
     */
    std::optional<DoublyConnectedEdgeList::edge_index> FindNextOnLine(
        const DoublyConnectedEdgeList& edgeList, DoublyConnectedEdgeList::edge_index edgeIndex, Float epsilon = EPSILON
    );

    /**
     * @brief Walk along the half edges that trace a line segment.
     * @param edgeList A doubly connected edge list.
     * @param edgeIndex An edge index in the edge list to start the walk.
     * @param function An object called with an edge index.
     * @param epsilon An error threshold used to find half edges on the line segment.
     */
    template <typename FunctionType>
    void WalkLine(
        const DoublyConnectedEdgeList& edgeList, DoublyConnectedEdgeList::edge_index edgeIndex, FunctionType&& function,
        Float epsilon = EPSILON
    ) {
        std::optional currentEdgeIndex = edgeIndex;
        while (currentEdgeIndex.has_value()) {
            function(currentEdgeIndex.value());
            currentEdgeIndex = FindNextOnLine(edgeList, currentEdgeIndex.value(), epsilon);
        }
    }
} // namespace compg