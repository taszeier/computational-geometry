#include "triangulation/MonotonePolygonTriangulator.hpp"
#include "math/Geometry.hpp"
#include "sweep_line/SweepLine.hpp"

#include <iterator>
#include <stack>
#include <unordered_set>
#include <vector>

namespace compg {

    using vertex_index = DoublyConnectedEdgeList::vertex_index;
    using edge_index = DoublyConnectedEdgeList::edge_index;

    namespace details {
        struct Cursor {
            vertex_index VertexIndex;
            edge_index EdgeIndex;
        };

        auto FindMinMax(const DoublyConnectedEdgeList& polygon, edge_index startEdgeIndex) {
            // TODO: don't iterate over all vertices. Go up to find max, go down to find min

            Cursor min{.VertexIndex = polygon.GetOriginIndex(startEdgeIndex), .EdgeIndex = startEdgeIndex};
            Cursor max{.VertexIndex = polygon.GetOriginIndex(startEdgeIndex), .EdgeIndex = startEdgeIndex};

            auto currentEdgeIndex = startEdgeIndex;
            do {
                const auto currentVertexIndex = polygon.GetOriginIndex(currentEdgeIndex);
                if (VertexBelowComparator::Compare(
                        polygon.GetVertex(currentVertexIndex).Vertex, polygon.GetVertex(min.VertexIndex).Vertex
                    )) {

                    min = {.VertexIndex = currentVertexIndex, .EdgeIndex = currentEdgeIndex};
                }
                if (VertexAboveComparator::Compare(
                        polygon.GetVertex(currentVertexIndex).Vertex, polygon.GetVertex(max.VertexIndex).Vertex
                    )) {
                    max = {.VertexIndex = currentVertexIndex, .EdgeIndex = currentEdgeIndex};
                }

                currentEdgeIndex = polygon.GetNextIndex(currentEdgeIndex);
            } while (currentEdgeIndex != startEdgeIndex);

            return std::pair{min, max};
        }

        auto CollectChain(const DoublyConnectedEdgeList& polygon, Cursor begin, vertex_index end) {
            std::vector<vertex_index> chain;
            auto cursor = begin;
            while (cursor.VertexIndex != end) {
                chain.push_back(cursor.VertexIndex);
                cursor
                    = {.VertexIndex = polygon.GetDestinationIndex(cursor.EdgeIndex),
                       .EdgeIndex = polygon.GetNextIndex(cursor.EdgeIndex)};
            }
            chain.push_back(cursor.VertexIndex);

            return chain;
        }

        enum struct ChainSide {
            Left,
            Right
        };

        class MonotonePolygonTriangulatorState {
        public:
            MonotonePolygonTriangulatorState(DoublyConnectedEdgeList& polygon, edge_index startEdgeIndex)
                : Polygon(polygon) {
                const auto [min, max] = FindMinMax(Polygon, startEdgeIndex);
                LeftChain = CollectChain(Polygon, max, min.VertexIndex);
                std::ranges::copy(LeftChain, std::inserter(LeftChainSet, LeftChainSet.end()));

                RightChain = CollectChain(polygon, min, max.VertexIndex) | std::views::reverse
                             | std::ranges::to<std::vector>();
                std::ranges::copy(RightChain, std::inserter(RightChainSet, RightChainSet.end()));
            }

            void Run() {
                auto rightChainTrimmed = RightChain | std::views::take(RightChain.size() - 1) | std::views::drop(1);
                std::vector<vertex_index> sortedVertices(LeftChain.size() + rightChainTrimmed.size());
                const auto projector = [this](vertex_index i) { return Polygon.GetVertex(i).Vertex; };
                std::ranges::merge(
                    LeftChain, rightChainTrimmed, sortedVertices.begin(), VertexAboveComparator{}, projector, projector
                );
                std::stack<vertex_index> stack;
                stack.push(sortedVertices.at(0));
                stack.push(sortedVertices.at(1));

                std::ranges::for_each(
                    sortedVertices | std::views::take(sortedVertices.size() - 1) | std::views::drop(2),
                    [this, &stack](vertex_index vertexIndex) { HandleVertex(vertexIndex, stack); }
                );

                stack.pop();
                while (stack.size() > 1) {
                    Polygon.InsertEdge(sortedVertices.back(), stack.top());
                    stack.pop();
                }
            }

            void HandleVertex(vertex_index vertexIndex, std::stack<vertex_index>& stack) const {
                if (FindChainSide(vertexIndex) != FindChainSide(stack.top())) {
                    const auto previousVertexIndex = stack.top();
                    while (stack.size() > 1) {
                        const auto top = stack.top();
                        stack.pop();
                        Polygon.InsertEdge(vertexIndex, top);
                    }
                    stack.pop();
                    stack.push(previousVertexIndex);
                    stack.push(vertexIndex);
                } else {
                    auto v1 = stack.top();
                    stack.pop();
                    while (!stack.empty() && IsEdgeInsidePolygon(vertexIndex, stack.top(), v1)) {
                        Polygon.InsertEdge(vertexIndex, stack.top());
                        v1 = stack.top();
                        stack.pop();
                    }

                    stack.push(v1);
                    stack.push(vertexIndex);
                }
            }

            bool IsEdgeInsidePolygon(vertex_index lower, vertex_index upper, vertex_index previousIndex) const {
                const auto side = FindSide(
                    Polygon.GetVertex(lower).Vertex, Polygon.GetVertex(upper).Vertex,
                    Polygon.GetVertex(previousIndex).Vertex
                );
                switch (FindChainSide(previousIndex)) {
                case ChainSide::Left:
                    return side == PointSide::Negative;
                case ChainSide::Right:
                    return side == PointSide::Positive;
                }
                COMPG_THROW("Received unexpected chain side value");
            }

            ChainSide FindChainSide(vertex_index vertexIndex) const {
                if (LeftChainSet.contains(vertexIndex)) {
                    return ChainSide::Left;
                }
                if (RightChainSet.contains(vertexIndex)) {
                    return ChainSide::Right;
                }
                COMPG_THROW("Received vertex index that is in neither chain");
            }

        private:
            DoublyConnectedEdgeList& Polygon;
            std::vector<vertex_index> LeftChain;
            std::unordered_set<vertex_index> LeftChainSet;
            std::vector<vertex_index> RightChain;
            std::unordered_set<vertex_index> RightChainSet;
        };
    } // namespace details

    void MonotonePolygonTriangulator::Triangulate(
        DoublyConnectedEdgeList& polygon, DoublyConnectedEdgeList::edge_index edgeIndex
    ) const {
        details::MonotonePolygonTriangulatorState state{polygon, edgeIndex};
        state.Run();
    }
} // namespace compg
