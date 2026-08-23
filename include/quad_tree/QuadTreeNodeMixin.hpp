#pragma once

#include "common/Common.hpp"
#include "quad_tree/Directions.hpp"

#include <optional>

namespace compg {

    template <typename NodeType, typename InternalNodeType, typename LeafNodeType>
    class QuadTreeNodeMixin {
        using node_type = NodeType;
        using internal_node_type = InternalNodeType;
        using leaf_node_type = LeafNodeType;

    public:
        [[nodiscard]] std::optional<OrdinalDirection> GetChildType() const {
            const auto self = static_cast<const node_type*>(this);
            if (self->Parent == nullptr) {
                return std::nullopt;
            }

            return std::visit(
                Overloads{
                    [this](const internal_node_type& p) {
                        for (const auto& d : GetAllOrdinalDirections()) {
                            if (p.GetChild(d).get() == this) {
                                return d;
                            }
                        }
                        COMPG_THROW("The child's `Parent` parameter points to incorrect parent");
                    },
                    [](const leaf_node_type&) -> OrdinalDirection { COMPG_THROW("The parent node cannot be a leaf"); }
                },
                self->Parent->State
            );
        }

        [[nodiscard]] node_type* FindNeighbor(CardinalDirection cardinalDirection) const {
            const auto childType = GetChildType();
            const auto self = static_cast<const node_type*>(this);

            return childType
                .transform([cardinalDirection, self](OrdinalDirection ordinalDirection) {
                    return std::visit(
                        Overloads{
                            [ordinalDirection, cardinalDirection, self](const internal_node_type& p) -> node_type* {
                                if (!HasComponent(ordinalDirection, cardinalDirection)) {
                                    return p.GetChild(AddComponent(ordinalDirection, cardinalDirection)).get();
                                }

                                const auto parentNeighbor = self->Parent->FindNeighbor(cardinalDirection);
                                if (parentNeighbor == nullptr)
                                    return nullptr;
                                return std::visit(
                                    Overloads{
                                        [ordinalDirection,
                                         cardinalDirection](const internal_node_type& n) -> node_type* {
                                            return n.GetChild(RemoveComponent(ordinalDirection, cardinalDirection))
                                                .get();
                                        },
                                        [parentNeighbor](const leaf_node_type&) -> node_type* { return parentNeighbor; }
                                    },
                                    parentNeighbor->State
                                );
                            },
                            [](const leaf_node_type&) -> node_type* { COMPG_THROW("The parent node cannot be a leaf"); }
                        },
                        self->Parent->State
                    );
                })
                .value_or(nullptr);
        }
    };
} // namespace compg