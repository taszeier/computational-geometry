#pragma once

#include "common/Error.hpp"
#include <array>

namespace compg {
    enum class CardinalDirection {
        North,
        East,
        South,
        West
    };

    constexpr auto GetAllCardinalDirections() {
        constexpr std::array directions
            = {CardinalDirection::North, CardinalDirection::East, CardinalDirection::South, CardinalDirection::West};
        return directions;
    }

    enum class OrdinalDirection {
        NorthEast,
        NorthWest,
        SouthWest,
        SouthEast
    };

    constexpr auto GetAllOrdinalDirections() {
        constexpr std::array directions = {
            OrdinalDirection::NorthEast,
            OrdinalDirection::NorthWest,
            OrdinalDirection::SouthWest,
            OrdinalDirection::SouthEast,
        };
        return directions;
    }

    constexpr CardinalDirection Flip(CardinalDirection cardinalDirection) {
        auto i = static_cast<std::size_t>(cardinalDirection);
        i = (i + 2) % 4;
        return static_cast<CardinalDirection>(i);
    }

    bool HasComponent(OrdinalDirection ordinalDirection, CardinalDirection cardinalDirection);
    std::tuple<CardinalDirection, CardinalDirection> Decompose(OrdinalDirection ordinalDirection);

    constexpr OrdinalDirection Compose(const std::tuple<CardinalDirection, CardinalDirection>& decomposition) {
        const auto& c0 = std::get<0>(decomposition);
        const auto& c1 = std::get<1>(decomposition);
        if (c0 == CardinalDirection::North) {
            if (c1 == CardinalDirection::East) {
                return OrdinalDirection::NorthEast;
            }
            if (c1 == CardinalDirection::West) {
                return OrdinalDirection::NorthWest;
            }
        } else if (c0 == CardinalDirection::South) {
            if (c1 == CardinalDirection::East) {
                return OrdinalDirection::SouthEast;
            }
            if (c1 == CardinalDirection::West) {
                return OrdinalDirection::SouthWest;
            }
        }

        COMPG_THROW("Received invalid decomposition");
    }

    OrdinalDirection AddComponent(OrdinalDirection ordinalDirection, CardinalDirection cardinalDirection);

    inline OrdinalDirection RemoveComponent(OrdinalDirection ordinalDirection, CardinalDirection cardinalDirection) {
        return AddComponent(ordinalDirection, Flip(cardinalDirection));
    }
} // namespace compg