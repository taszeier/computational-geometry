#pragma once

#include <optional>

namespace compg {
    struct SiteEventId {
        std::size_t Id;

        bool operator==(const SiteEventId& other) const {
            return Id == other.Id;
        }
    };

    struct Arc {
        std::optional<SiteEventId> LeftSite;
        SiteEventId MiddleSite{};
        std::optional<SiteEventId> RightSite;

        bool operator==(const Arc& other) const {
            return LeftSite == other.LeftSite && MiddleSite == other.MiddleSite && RightSite == other.RightSite;
        }
    };

    struct CircleEventId {
        Arc Arc;

        bool operator==(const CircleEventId& other) const {
            return Arc == other.Arc;
        }
    };

    struct BreakPoint {
        SiteEventId LeftSite;
        SiteEventId RightSite;

        bool operator==(const BreakPoint& other) const noexcept {
            return LeftSite == other.LeftSite && RightSite == other.RightSite;
        }
    };
} // namespace compg

namespace std {
    template <>
    struct hash<compg::SiteEventId> {
        size_t operator()(compg::SiteEventId e) const noexcept {
            size_t seed{0};
            compg::hash_combine(seed, e.Id);
            return seed;
        }
    };

    template <>
    struct hash<compg::Arc> {
        size_t operator()(const compg::Arc& arc) const noexcept {
            size_t seed{0};
            compg::hash_combine(seed, arc.LeftSite);
            compg::hash_combine(seed, arc.MiddleSite);
            compg::hash_combine(seed, arc.RightSite);
            return seed;
        }
    };

    template <>
    struct hash<compg::CircleEventId> {
        size_t operator()(const compg::CircleEventId& e) const noexcept {
            size_t seed{0};
            compg::hash_combine(seed, e.Arc);
            return seed;
        }
    };

    template <>
    struct hash<compg::BreakPoint> {
        size_t operator()(const compg::BreakPoint& breakPoint) const noexcept {
            size_t seed{0};
            compg::hash_combine(seed, breakPoint.LeftSite);
            compg::hash_combine(seed, breakPoint.RightSite);
            return seed;
        }
    };
} // namespace std