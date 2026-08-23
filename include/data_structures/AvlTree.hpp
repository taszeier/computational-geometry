#pragma once

#include <memory>
#include <utility>

#include "AvlTree.hpp"
#include "common/Common.hpp"
#include "common/Error.hpp"

namespace compg {

    template <typename ValueType>
    struct AvlTreeNode {
        ValueType Value;
        std::unique_ptr<AvlTreeNode> LeftChild = nullptr;
        std::unique_ptr<AvlTreeNode> RightChild = nullptr;
        AvlTreeNode* Parent = nullptr;
        std::size_t Height = 1UZ;

        void SetLeftChild(std::unique_ptr<AvlTreeNode> node) {
            LeftChild = std::move(node);
            if (LeftChild != nullptr) {
                LeftChild->Parent = this;
            }
        }

        void SetRightChild(std::unique_ptr<AvlTreeNode> node) {
            RightChild = std::move(node);
            if (RightChild != nullptr) {
                RightChild->Parent = this;
            }
        }

        std::unique_ptr<AvlTreeNode> ReplaceLeftChild(std::unique_ptr<AvlTreeNode> newChild) {
            auto oldChild = std::exchange(LeftChild, nullptr);
            if (oldChild != nullptr) {
                oldChild->Parent = nullptr;
            }
            SetLeftChild(std::move(newChild));
            return oldChild;
        }

        std::unique_ptr<AvlTreeNode> ReplaceRightChild(std::unique_ptr<AvlTreeNode> newChild) {
            auto oldChild = std::exchange(RightChild, nullptr);
            if (oldChild != nullptr) {
                oldChild->Parent = nullptr;
            }
            SetRightChild(std::move(newChild));
            return oldChild;
        }

        std::unique_ptr<AvlTreeNode> ReplaceChild(AvlTreeNode* currentChild, std::unique_ptr<AvlTreeNode> newChild) {
            COMPG_ASSERT(currentChild != nullptr, "Expected a valid pointer");
            if (LeftChild.get() == currentChild) {
                return ReplaceLeftChild(std::move(newChild));
            }
            if (RightChild.get() == currentChild) {
                return ReplaceRightChild(std::move(newChild));
            }
            COMPG_THROW("Parameter `currentChild` is not a child of this node");
        }

        std::unique_ptr<AvlTreeNode> DisownLeftChild() {
            return ReplaceLeftChild(nullptr);
        }

        std::unique_ptr<AvlTreeNode> DisownRightChild() {
            return ReplaceRightChild(nullptr);
        }

        std::unique_ptr<AvlTreeNode> DisownChild(AvlTreeNode* child) {
            COMPG_ASSERT(child != nullptr, "Expected a valid pointer");
            if (LeftChild.get() == child) {
                return DisownLeftChild();
            }
            if (RightChild.get() == child) {
                return DisownRightChild();
            }
            COMPG_THROW("Parameter `child` is not a child of this node");
        }

        [[nodiscard]] constexpr bool IsLeaf() const {
            return LeftChild == nullptr && RightChild == nullptr;
        }
    };

    namespace details {

        template <typename ValueType>
        std::size_t GetHeight(const std::unique_ptr<AvlTreeNode<ValueType>>& nodePtr) {
            return nodePtr == nullptr ? 0 : nodePtr->Height;
        }

        template <typename ValueType>
        void UpdateHeight(AvlTreeNode<ValueType>& node) {
            node.Height = 1 + std::max(GetHeight(node.LeftChild), GetHeight(node.RightChild));
        }

        template <typename ValueType>
        int FindBalance(const AvlTreeNode<ValueType>& node) {
            return GetHeight(node.LeftChild) - GetHeight(node.RightChild);
        }

        template <typename ValueType>
        std::unique_ptr<AvlTreeNode<ValueType>>& GetSelfFromParent(AvlTreeNode<ValueType>* childPtr) {
            COMPG_ASSERT(childPtr != nullptr, "Expected a valid pointer");
            COMPG_ASSERT(childPtr->Parent != nullptr, "Expected an internal node");

            if (childPtr->Parent->LeftChild.get() == childPtr) {
                return childPtr->Parent->LeftChild;
            }
            if (childPtr->Parent->RightChild.get() == childPtr) {
                return childPtr->Parent->RightChild;
            }
            COMPG_THROW("The node is not a child of its parent");
        }

        template <typename ValueType>
        std::unique_ptr<AvlTreeNode<ValueType>> RotateRight(std::unique_ptr<AvlTreeNode<ValueType>> nodePtr) {
            COMPG_ASSERT(nodePtr->LeftChild != nullptr, "The root must have a left child");

            auto v1 = std::move(nodePtr);
            auto v2 = std::move(v1->LeftChild);
            auto t3 = std::move(v2->RightChild);

            v1->SetLeftChild(std::move(t3));
            UpdateHeight(*v1);

            v2->Parent = v1->Parent;
            v2->SetRightChild(std::move(v1));
            UpdateHeight(*v2);

            return v2;
        }

        template <typename ValueType>
        std::unique_ptr<AvlTreeNode<ValueType>> RotateLeft(std::unique_ptr<AvlTreeNode<ValueType>> nodePtr) {
            COMPG_ASSERT(nodePtr->RightChild != nullptr, "The root must have a right child");

            auto v1 = std::move(nodePtr);
            auto v2 = std::move(v1->RightChild);
            auto t2 = std::move(v2->LeftChild);

            v1->SetRightChild(std::move(t2));
            UpdateHeight(*v1);

            v2->Parent = v1->Parent;
            v2->SetLeftChild(std::move(v1));
            UpdateHeight(*v2);

            return v2;
        }

        template <typename ValueType>
        std::unique_ptr<AvlTreeNode<ValueType>> RotateIfNecessary(std::unique_ptr<AvlTreeNode<ValueType>> nodePtr) {
            COMPG_ASSERT(nodePtr != nullptr, "Expected a valid pointer");
            UpdateHeight(*nodePtr);
            const auto balance = FindBalance(*nodePtr);

            if (balance > 1) {
                COMPG_ASSERT(
                    nodePtr->LeftChild != nullptr, "The left child cannot be null if the balance is greater than 1"
                );
                if (FindBalance(*nodePtr->LeftChild) >= 0) {
                    return RotateRight(std::move(nodePtr));
                }
                nodePtr->SetLeftChild(RotateLeft(std::move(nodePtr->LeftChild)));
                return RotateRight(std::move(nodePtr));
            }
            if (balance < -1) {
                COMPG_ASSERT(
                    nodePtr->RightChild != nullptr, "The right child cannot be null if the balance is less than -1"
                );
                if (FindBalance(*nodePtr->RightChild) <= 0) {
                    return RotateLeft(std::move(nodePtr));
                }
                nodePtr->SetRightChild(RotateRight(std::move(nodePtr->RightChild)));
                return RotateLeft(std::move(nodePtr));
            }

            return nodePtr;
        }

        template <typename ValueType>
        std::unique_ptr<AvlTreeNode<ValueType>>
        BalanceUpwards(std::unique_ptr<AvlTreeNode<ValueType>> rootPtr, AvlTreeNode<ValueType>* balanceStartPtr) {
            if (rootPtr == nullptr) {
                return nullptr;
            }

            COMPG_ASSERT(balanceStartPtr != nullptr, "Expected a valid pointer");

            auto balancePtr = balanceStartPtr;
            while (balancePtr->Parent != nullptr) {
                auto nextBalancePtr = balancePtr->Parent;
                auto& child = GetSelfFromParent(balancePtr);
                child = RotateIfNecessary(std::move(child));
                balancePtr = nextBalancePtr;
            }

            return RotateIfNecessary(std::move(rootPtr));
        }

        template <typename TreeValueType, typename ValueType>
        std::tuple<std::unique_ptr<AvlTreeNode<TreeValueType>>, AvlTreeNode<TreeValueType>*> InsertBefore(
            std::unique_ptr<AvlTreeNode<TreeValueType>> rootPtr, ValueType&& value,
            AvlTreeNode<TreeValueType>* insertPtr
        ) {
            if (rootPtr == nullptr) {
                auto newNode = std::make_unique<AvlTreeNode<TreeValueType>>(std::forward<ValueType>(value));
                const auto newNodePtr = newNode.get();
                return std::make_tuple(std::move(newNode), newNodePtr);
            }
            // COMPG_ASSERT(insertPtr != nullptr, "Expected a valid pointer");
            if (insertPtr != nullptr && insertPtr->LeftChild == nullptr) {
                auto newNode = std::make_unique<AvlTreeNode<TreeValueType>>(std::forward<ValueType>(value));
                const auto newNodePtr = newNode.get();
                insertPtr->SetLeftChild(std::move(newNode));
                return std::tuple{BalanceUpwards(std::move(rootPtr), insertPtr), newNodePtr};
            }

            auto nodePtr = insertPtr == nullptr ? rootPtr.get() : insertPtr->LeftChild.get();
            while (nodePtr->RightChild != nullptr) {
                nodePtr = nodePtr->RightChild.get();
            }

            auto newNode = std::make_unique<AvlTreeNode<TreeValueType>>(std::forward<ValueType>(value));
            const auto newNodePtr = newNode.get();
            nodePtr->SetRightChild(std::move(newNode));
            return std::tuple{BalanceUpwards(std::move(rootPtr), nodePtr), newNodePtr};
        }

        template <
            typename TreeValueType, typename ValueType, typename ComparatorType = Less,
            typename ProjectorType = Identity>
        std::tuple<std::unique_ptr<AvlTreeNode<TreeValueType>>, AvlTreeNode<TreeValueType>*, bool> Insert(
            std::unique_ptr<AvlTreeNode<TreeValueType>> rootPtr, ValueType&& value, ComparatorType comparator = {},
            ProjectorType projector = {}
        ) {

            if (rootPtr == nullptr) {
                auto newNode = std::make_unique<AvlTreeNode<TreeValueType>>(std::forward<ValueType>(value));
                const auto newNodePtr = newNode.get();
                return std::make_tuple(std::move(newNode), newNodePtr, true);
            }

            auto nodePtr = rootPtr.get();
            decltype(nodePtr) parentPtr = nullptr;
            bool found = false;
            decltype(auto) insertProjection = projector(value);

            while (nodePtr != nullptr && !found) {
                decltype(auto) nodeProjection = projector(nodePtr->Value);
                if (comparator(insertProjection, nodeProjection)) {
                    parentPtr = nodePtr;
                    nodePtr = nodePtr->LeftChild.get();
                } else if (comparator(nodeProjection, insertProjection)) {
                    parentPtr = nodePtr;
                    nodePtr = nodePtr->RightChild.get();
                } else {
                    found = true;
                }
            }

            if (found) {
                return std::make_tuple(std::move(rootPtr), nodePtr, false);
            }

            auto newNode = std::make_unique<AvlTreeNode<TreeValueType>>(std::forward<ValueType>(value));
            const auto newNodePtr = newNode.get();

            if (comparator(projector(newNode->Value), projector(parentPtr->Value))) {
                parentPtr->SetLeftChild(std::move(newNode));
            } else {
                parentPtr->SetRightChild(std::move(newNode));
            }
            return std::make_tuple(BalanceUpwards(std::move(rootPtr), parentPtr), newNodePtr, true);
        }

        template <typename TreeValueType>
        std::unique_ptr<AvlTreeNode<TreeValueType>>
        Delete(std::unique_ptr<AvlTreeNode<TreeValueType>> rootPtr, AvlTreeNode<TreeValueType>* deletePtr) {
            COMPG_ASSERT(deletePtr != nullptr, "Expected a valid pointer");

            auto balancePtr = deletePtr->Parent;
            if (deletePtr->LeftChild != nullptr && deletePtr->RightChild != nullptr) {
                auto x = deletePtr->RightChild.get();
                while (x->LeftChild != nullptr) {
                    x = x->LeftChild.get();
                }
                balancePtr = x->Parent;
                auto x_u = x->Parent->ReplaceChild(x, x->DisownRightChild());
                x_u->SetLeftChild(deletePtr->DisownLeftChild());
                x_u->SetRightChild(deletePtr->DisownRightChild());
                if (deletePtr->Parent != nullptr) {
                    deletePtr->Parent->ReplaceChild(deletePtr, std::move(x_u));
                } else {
                    return RotateIfNecessary(std::move(x_u));
                }
            } else if (deletePtr->LeftChild == nullptr) {
                if (deletePtr->Parent == nullptr) {
                    return deletePtr->DisownRightChild();
                }

                deletePtr->Parent->ReplaceChild(deletePtr, deletePtr->DisownRightChild());
            } else {
                if (deletePtr->Parent == nullptr) {
                    return deletePtr->DisownLeftChild();
                }
                deletePtr->Parent->ReplaceChild(deletePtr, deletePtr->DisownLeftChild());
            }

            return BalanceUpwards(std::move(rootPtr), balancePtr);
        }
    } // namespace details

    /**
     * @brief A balanced binary tree supporting low-level operations.
     * @tparam TreeValueType The type of the values to store in the tree.
     */
    template <typename TreeValueType>
    class AvlTree {
    public:
        class AvlTreeIterator {
        public:
            using iterator_category = std::bidirectional_iterator_tag;
            using value_type = TreeValueType;
            using difference_type = std::ptrdiff_t;
            using pointer = TreeValueType*;
            using reference = const TreeValueType&;

            reference operator*() const {
                return Node->Value;
            }

            pointer operator->() const {
                return &Node->Value;
            }

            AvlTreeIterator& operator++() {
                if (Node != nullptr) {
                    if (Node->RightChild != nullptr) {
                        auto node = Node->RightChild.get();
                        while (node->LeftChild != nullptr) {
                            node = node->LeftChild.get();
                        }
                        Node = node;
                    } else {
                        auto node = Node;
                        auto parent = node->Parent;
                        while (parent != nullptr && parent->RightChild.get() == node) {
                            node = parent;
                            parent = node->Parent;
                        }
                        Node = parent;
                    }
                }
                return *this;
            }

            AvlTreeIterator operator++(int) {
                AvlTreeIterator result(*this);
                operator++();
                return result;
            }

            AvlTreeIterator& operator--() {
                if (Node == nullptr) {
                    auto node = Tree->Root.get();
                    while (node != nullptr && node->RightChild != nullptr) {
                        node = node->RightChild.get();
                    }
                    Node = node;
                    return *this;
                }
                if (Node->LeftChild != nullptr) {
                    auto node = Node->LeftChild.get();
                    while (node->RightChild != nullptr) {
                        node = node->RightChild.get();
                    }
                    Node = node;
                } else {
                    auto node = Node;
                    auto parent = node->Parent;
                    while (parent != nullptr && parent->LeftChild.get() == node) {
                        node = parent;
                        parent = node->Parent;
                    }
                    Node = parent;
                }
                return *this;
            }

            AvlTreeIterator operator--(int) {
                AvlTreeIterator result(*this);
                operator--();
                return result;
            }

            bool operator==(const AvlTreeIterator& other) const {
                return Node == other.Node;
            }

            bool operator!=(const AvlTreeIterator& other) const {
                return !(*this == other);
            }

            explicit AvlTreeIterator(const AvlTree<TreeValueType>* tree, AvlTreeNode<TreeValueType>* node = nullptr)
                : Tree(tree)
                , Node(node) {}

            static AvlTreeIterator CreateBegin(const AvlTree* tree) {
                const auto root = tree->Root.get();
                if (root == nullptr) {
                    return AvlTreeIterator{tree, nullptr};
                }

                auto node = root;
                while (node->LeftChild != nullptr) {
                    node = node->LeftChild.get();
                }
                return AvlTreeIterator{tree, node};
            }

            static AvlTreeIterator CreateEnd(const AvlTree* tree) {
                return AvlTreeIterator{tree, nullptr};
            }

            friend class AvlTree;

        private:
            const AvlTree* Tree;
            AvlTreeNode<TreeValueType>* Node = nullptr;
        };

        using node_type = AvlTreeNode<TreeValueType>;
        using pointer_type = std::unique_ptr<node_type>;
        using iterator = AvlTreeIterator;

        /**
         * @brief Insert a value into the tree.
         * @return An iterator to the value in the tree and whether the insertion took place.
         */
        template <typename ValueType, typename ComparatorType = Less, typename ProjectorType = Identity>
        std::pair<iterator, bool>
        Insert(ValueType&& value, ComparatorType comparator = {}, ProjectorType projector = {}) {
            auto [root, insertPtr, inserted] = details::Insert<TreeValueType>(
                std::move(Root), std::forward<ValueType>(value), comparator, projector
            );
            Root = std::move(root);
            return std::pair{iterator{this, insertPtr}, inserted};
        }

        /**
         * @brief Insert a value into the tree before a given position.
         * @param value The value to insert.
         * @param it The insert position.
         * @return An iterator to the inserted value.
         */
        template <typename ValueType>
        auto InsertBefore(ValueType&& value, AvlTreeIterator it) {
            COMPG_ASSERT(it.Tree == this, "Received an iterator to a node in a different tree");
            auto [root, insertPtr]
                = details::InsertBefore<TreeValueType>(std::move(Root), std::forward<ValueType>(value), it.Node);
            Root = std::move(root);
            return iterator{this, insertPtr};
        }

        /**
         * @brief Erase a value from the tree.
         * @param it An iterator to the value.
         * @return Whether the erasure took place.
         */
        bool Erase(AvlTreeIterator it) {
            COMPG_ASSERT(it.Tree == this, "Received an iterator to a node in a different tree");
            if (it != end()) {
                Root = details::Delete(std::move(Root), it.Node);
                return true;
            }
            return false;
        }

        template <typename ValueType, typename ComparatorType = Less, typename ProjectorType = Identity>
        bool Erase(const ValueType& value, ComparatorType comparator = {}, ProjectorType projector = {}) {
            return Erase(Find(value, comparator, projector));
        }

        /**
         * @brief Find the left-most position that is not ordered before the given value.
         */
        template <typename ValueType, typename ComparatorType = Less, typename ProjectorType = Identity>
        auto LowerBound(const ValueType& value, ComparatorType comparator = {}, ProjectorType projector = {}) const {
            auto node = Root.get();
            decltype(node) result = nullptr;

            while (node != nullptr) {
                decltype(auto) projection = projector(node->Value);
                if (!comparator(projection, value)) {
                    result = node;
                    node = node->LeftChild.get();
                } else {
                    node = node->RightChild.get();
                }
            }
            return AvlTreeIterator{this, result};
        }

        /**
         * @brief Find the left-most position that is ordered after the given value.
         */
        template <typename ValueType, typename ComparatorType = Less, typename ProjectorType = Identity>
        auto UpperBound(const ValueType& value, ComparatorType comparator = {}, ProjectorType projector = {}) const {
            auto node = Root.get();
            decltype(node) result = nullptr;

            while (node != nullptr) {
                decltype(auto) projection = projector(node->Value);
                if (comparator(value, projection)) {
                    result = node;
                    node = node->LeftChild.get();
                } else {
                    node = node->RightChild.get();
                }
            }

            return AvlTreeIterator{this, result};
        }

        /**
         * @brief Find the first value in the tree that is equivalent to the given value.
         */
        template <typename ValueType, typename ComparatorType = Less, typename ProjectorType = Identity>
        auto Find(const ValueType& value, ComparatorType comparator = {}, ProjectorType projector = {}) const {
            auto node = Root.get();
            bool found = false;

            while (node != nullptr && !found) {
                decltype(auto) projection = projector(node->Value);
                if (comparator(value, projection)) {
                    node = node->LeftChild.get();
                } else if (comparator(projection, value)) {
                    node = node->RightChild.get();
                } else {
                    found = true;
                }
            }

            return found ? AvlTreeIterator{this, node} : AvlTreeIterator::CreateEnd(this);
        }

        auto begin() const {
            return AvlTreeIterator::CreateBegin(this);
        }

        auto end() const {
            return AvlTreeIterator::CreateEnd(this);
        }

        auto rbegin() const {
            return std::reverse_iterator<typename AvlTree::iterator>(end());
        }

        auto rend() const {
            return std::reverse_iterator<typename AvlTree::iterator>(begin());
        }

    private:
        pointer_type Root = nullptr;
    };
} // namespace compg
