#pragma once

#include <memory>

#include "binary_space_partition/ElementaryObject.hpp"
#include "math/primitives/Hyperplane.hpp"

namespace compg {
    template <std::size_t K>
    struct BspTreeLeafNode {
        std::vector<ElementaryObject<K>> Objects;
    };

    template <std::size_t K>
    struct BspTreeNode;

    template <std::size_t K>
    struct BspTreeInternalNode {
        Hyperplane<K> Split;
        std::vector<ElementaryObject<K>> Coplanar{};
        std::unique_ptr<BspTreeNode<K>> NegativeChild{};
        std::unique_ptr<BspTreeNode<K>> PositiveChild{};
    };

    template <std::size_t K>
    struct BspTreeNode {
        std::variant<BspTreeLeafNode<K>, BspTreeInternalNode<K>> State;
    };

    template <std::size_t K>
    struct BspTree {
        std::unique_ptr<BspTreeNode<K>> Root;
    };

    using BspTreeLeafNode2D = BspTreeLeafNode<2UZ>;
    using BspTreeInternalNode2D = BspTreeInternalNode<2UZ>;
    using BspTreeNode2D = BspTreeNode<2UZ>;
    using BspTree2D = BspTree<2UZ>;

    using BspTreeLeafNode3D = BspTreeLeafNode<3UZ>;
    using BspTreeInternalNode3D = BspTreeInternalNode<3UZ>;
    using BspTreeNode3D = BspTreeNode<3UZ>;
    using BspTree3D = BspTree<3UZ>;
} // namespace compg
