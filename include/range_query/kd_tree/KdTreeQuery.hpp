#pragma once

#include <vector>

#include "range_query/kd_tree/KdTree.hpp"
#include "range_query/kd_tree/QueryRegion.hpp"

namespace compg {
    namespace details {
        template <std::size_t K, typename ContainerType>
        void ReportSubtree(const typename KdTree<K>::pointer_type& node, ContainerType& output) {
            std::visit(
                Overloads{
                    [&output](const KdTreeLeafNode<K>& n) { output.push_back(n.Record); },
                    [&output](const KdTreeInternalNode<K>& n) {
                        ReportSubtree<K>(n.LeftChild, output);
                        ReportSubtree<K>(n.RightChild, output);
                    }
                },
                node->State
            );
        }

    } // namespace details

    template <std::size_t K>
    class KdTreeQuery {
    public:
        using pointer_type = KdTree<K>::pointer_type;
        using output_container_type = std::vector<VertexRecord<K>>;

        output_container_type Query(const KdTree<K>& tree, const QueryRegion<K>& queryRegion) const {
            return Query(tree.GetRoot(), {}, queryRegion);
        }

        output_container_type
        Query(const pointer_type& node, const KdTreeRegion<K>& treeRegion, const QueryRegion<K>& queryRegion) const {
            output_container_type result;
            Query(node, treeRegion, queryRegion, result);
            return result;
        }

    private:
        void Query(
            const pointer_type& node, const KdTreeRegion<K>& treeRegion, const QueryRegion<K>& queryRegion,
            output_container_type& output
        ) const {
            std::visit(
                Overloads{
                    [&output, &queryRegion](const KdTreeLeafNode<K>& n) {
                        if (queryRegion.Contains(n.Record.Value)) {
                            output.push_back(n.Record);
                        }
                    },
                    [&output, &queryRegion, this, &treeRegion](const KdTreeInternalNode<K>& n) {
                        const auto leftRegion = treeRegion.Intersected(n.Plane, Side::Negative);
                        if (queryRegion.Covers(leftRegion)) {
                            details::ReportSubtree<K>(n.LeftChild, output);
                        } else if (queryRegion.Intersects(leftRegion)) {
                            Query(n.LeftChild, leftRegion, queryRegion, output);
                        }
                        const auto rightRegion = treeRegion.Intersected(n.Plane, Side::Positive);
                        if (queryRegion.Covers(rightRegion)) {
                            details::ReportSubtree<K>(n.RightChild, output);
                        } else if (queryRegion.Intersects(rightRegion)) {
                            Query(n.RightChild, rightRegion, queryRegion, output);
                        }
                    }
                },
                node->State
            );
        }
    };
} // namespace compg
