#include "quad_tree/Directions.hpp"
#include "common/Vertex.hpp"
#include <map>
#include <ranges>

namespace compg {
    bool HasComponent(OrdinalDirection ordinalDirection, CardinalDirection cardinalDirection) {
        const std::map<OrdinalDirection, std::vector<CardinalDirection>> m
            = {{OrdinalDirection::NorthEast, {CardinalDirection::North, CardinalDirection::East}},
               {OrdinalDirection::NorthWest, {CardinalDirection::North, CardinalDirection::West}},
               {OrdinalDirection::SouthWest, {CardinalDirection::South, CardinalDirection::West}},
               {OrdinalDirection::SouthEast, {CardinalDirection::South, CardinalDirection::East}}};
        return std::ranges::contains(m.at(ordinalDirection), cardinalDirection);
    }

    std::tuple<CardinalDirection, CardinalDirection> Decompose(OrdinalDirection ordinalDirection) {
        const std::map<OrdinalDirection, std::tuple<CardinalDirection, CardinalDirection>> m
            = {{OrdinalDirection::NorthEast, std::make_tuple(CardinalDirection::North, CardinalDirection::East)},
               {OrdinalDirection::NorthWest, std::make_tuple(CardinalDirection::North, CardinalDirection::West)},
               {OrdinalDirection::SouthWest, std::make_tuple(CardinalDirection::South, CardinalDirection::West)},
               {OrdinalDirection::SouthEast, std::make_tuple(CardinalDirection::South, CardinalDirection::East)}};
        return m.at(ordinalDirection);
    }

    OrdinalDirection AddComponent(OrdinalDirection ordinalDirection, CardinalDirection cardinalDirection) {
        auto decomposition = Decompose(ordinalDirection);
        if (cardinalDirection == CardinalDirection::North || cardinalDirection == CardinalDirection::South) {
            std::get<0>(decomposition) = cardinalDirection;
        } else {
            std::get<1>(decomposition) = cardinalDirection;
        }
        return Compose(decomposition);
    }
} // namespace compg