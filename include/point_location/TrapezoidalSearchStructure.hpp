#pragma once

#include "common/Vertex.hpp"
#include "math/primitives/LineSegment.hpp"
#include "point_location/TrapezoidalMap.hpp"

namespace compg {
    struct TrapezoidalSearchLeafNode {
        std::size_t TrapezoidIndex;
    };

    struct TrapezoidalSearchNode;

    struct XNode {
        Vertex2D Vertex;
    };

    struct YNode {
        LineSegment2D Segment;
    };

    struct TrapezoidalSearchInternalNode {
        std::variant<XNode, YNode> Node;
        std::size_t LeftChild;
        std::size_t RightChild;
    };

    struct TrapezoidalSearchNode {
        std::variant<TrapezoidalSearchLeafNode, TrapezoidalSearchInternalNode> State;

        [[nodiscard]] constexpr bool IsLeaf() const {
            return State.index() == 0UZ;
        }
    };

    class TrapezoidalSearchStructure {
    public:
        using trapezoid_index = std::size_t;

        explicit TrapezoidalSearchStructure(trapezoid_index trapezoidIndex = 0) {
            Root = Nodes.size();
            Nodes.emplace_back(TrapezoidalSearchLeafNode{trapezoidIndex});
            TrapezoidLeafNodes[trapezoidIndex] = Root;
        }

        struct SearchResult {
            std::variant<trapezoid_index, LineSegment2D, Vertex2D> State;
        };

        /**
         * @brief Find the location of a vertex in the trapezoidal map.
         * @param vertex A query vertex.
         * @return The location of the query vertex. Either a trapezoid index, a line segment, or a vertex that is an
         * endpoint of a line segment.
         */
        SearchResult Search(const Vertex2D& vertex) const;

        /**
         * @brief Find the trapezoid in which the line segment starts.
         * @param segment The line segment.
         * @return The index of the trapezoid.
         */
        trapezoid_index FindTrapezoid(const LineSegment2D& segment) const;

        /**
         * @brief Find the trapezoids that intersect a line segment.
         * @param trapezoidalMap The trapezoidal map.
         * @param segment The line segment.
         * @return A vector of trapezoid indices that are intersected by the line segment. The indices are in the order
         * they are intersected, i.e., from left to right.
         */
        std::vector<trapezoid_index>
        FollowSegment(const TrapezoidalMap& trapezoidalMap, const LineSegment2D& segment) const;

        /**
         * @brief Update the search structure after a single trapezoid is split by a line segment.
         * @param trapezoidIndex The split trapezoid.
         * @param splitResult The result of the split obtained from the trapezoidal map.
         * @param segment The line segment that splits the trapezoid.
         */
        void UpdateAfterSplit(
            trapezoid_index trapezoidIndex, const TrapezoidalMap::SplitResult& splitResult, const LineSegment2D& segment
        );

        /**
         * @brief Update the search structure after trapezoids are split by a line segment.
         * @param trapezoidIndices The vector of indices of the split trapezoids ordered from left to right.
         * @param splitResult The result of the split obtained from the trapezoidal map.
         * @param segment The line segment that splits the trapezoids.
         */
        void UpdateAfterSplit(
            const std::vector<trapezoid_index>& trapezoidIndices, const TrapezoidalMap::SplitResult& splitResult,
            const LineSegment2D& segment
        );

    private:
        std::size_t Root;
        std::vector<TrapezoidalSearchNode> Nodes;
        std::unordered_map<trapezoid_index, std::size_t> TrapezoidLeafNodes;
    };
} // namespace compg
