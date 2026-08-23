#pragma once

#include "binary_space_partition/BspTree.hpp"
#include "math/primitives/HalfPlane.hpp"
#include <memory>

namespace compg {

    template <std::size_t K, typename FunctionType>
    struct BspLeafDfs {
        bool Call(std::unique_ptr<BspTreeNode<K>>& node) {
            return operator()(node);
        }

        bool operator()(std::unique_ptr<BspTreeNode<K>>& node) {
            return std::visit(
                Overloads{
                    [this](BspTreeInternalNode<K>& internalNode) {
                        Planes.emplace_back(internalNode.Split);
                        if (Call(internalNode.PositiveChild)) {
                            Planes.pop_back();
                            return true;
                        }
                        Planes.back().Flip();
                        const auto result = Call(internalNode.NegativeChild);
                        Planes.pop_back();
                        return result;
                    },
                    [this, &node](const BspTreeLeafNode<K>&) { return Function(Planes, node); }
                },
                node->State
            );
        }

        bool Call(const std::unique_ptr<BspTreeNode<K>>& node) {
            return operator()(node);
        }

        bool operator()(const std::unique_ptr<BspTreeNode<K>>& node) {
            return std::visit(
                Overloads{
                    [this](const BspTreeInternalNode<K>& internalNode) {
                        Planes.emplace_back(internalNode.Split);
                        if (Call(internalNode.PositiveChild)) {
                            Planes.pop_back();
                            return true;
                        }
                        Planes.back().Flip();
                        const auto result = Call(internalNode.NegativeChild);
                        Planes.pop_back();
                        return result;
                    },
                    [this, &node](const BspTreeLeafNode<K>&) { return Function(Planes, node); }
                },
                node->State
            );
        }

        FunctionType Function;
        std::vector<HalfPlane<Hyperplane<K>>> Planes{};
    };
} // namespace compg
