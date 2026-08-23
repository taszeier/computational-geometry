#include "point_location/TrapezoidalMap.hpp"
#include "math/Geometry.hpp"
#include "point_location/Common.hpp"

namespace compg {

    TrapezoidalMap::TrapezoidalMap(const Box2D& boundingBox)
        : BoundingBox(boundingBox) {
        const auto [v0, v1, v2, v3] = GetCorners(boundingBox);

        Trapezoids.emplace_back(TrapezoidRecord{v3, v1, LineSegment2D{v2, v3}, LineSegment2D{v0, v1}});
        Vertices.insert(v0);
        Vertices.insert(v1);
        Vertices.insert(v2);
        Vertices.insert(v3);
    }

    using SplitResult = TrapezoidalMap::SplitResult;

    SplitResult TrapezoidalMap::Split(const LineSegment2D& segment, trapezoid_index trapezoidIndex) {
        COMPG_ASSERT(
            trapezoidIndex < NumTrapezoids(), CreateOutOfRangeMessage("trapezoidIndex", trapezoidIndex, NumTrapezoids())
        );

        const auto [v0, v1] = GetEndPoints(segment);
        const auto trapezoid = Trapezoids.at(trapezoidIndex);

        TrapezoidRecord t1{v0, v1, trapezoid.TopSegment, segment};
        const auto t1Index = trapezoidIndex;
        TrapezoidRecord t2{v0, v1, segment, trapezoid.BottomSegment};
        const auto t2Index = Trapezoids.size();

        std::optional<TrapezoidRecord> maybet3
            = AreEqual(trapezoid.LeftPoint, v0)
                  ? std::nullopt
                  : std::optional{
                        TrapezoidRecord{trapezoid.LeftPoint, v0, trapezoid.TopSegment, trapezoid.BottomSegment}
                    };
        const auto maybet3Index = maybet3.has_value() ? std::optional{t2Index + 1} : std::nullopt;

        std::optional<TrapezoidRecord> maybet4
            = AreEqual(trapezoid.RightPoint, v1)
                  ? std::nullopt
                  : std::optional{
                        TrapezoidRecord{v1, trapezoid.RightPoint, trapezoid.TopSegment, trapezoid.BottomSegment}
                    };
        const auto maybet4Index
            = maybet4.has_value() ? std::optional(maybet3Index.value_or(t2Index) + 1) : std::nullopt;

        t1.UpperLeft = maybet3Index.has_value() ? maybet3Index : trapezoid.UpperLeft;
        t1.UpperRight = maybet4Index.has_value() ? maybet4Index : trapezoid.UpperRight;
        t2.LowerLeft = maybet3Index.has_value() ? maybet3Index : trapezoid.LowerLeft;
        t2.LowerRight = maybet4Index.has_value() ? maybet4Index : trapezoid.LowerRight;
        if (maybet3.has_value()) {
            maybet3->UpperLeft = trapezoid.UpperLeft;
            maybet3->LowerLeft = trapezoid.LowerLeft;
            maybet3->UpperRight = t1Index;
            maybet3->LowerRight = t2Index;
        }

        if (maybet4.has_value()) {
            maybet4->UpperLeft = t1Index;
            maybet4->LowerLeft = t2Index;
            maybet4->UpperRight = trapezoid.UpperRight;
            maybet4->LowerRight = trapezoid.LowerRight;
            ;
        }

        if (auto upperRight = Trapezoids.at(trapezoidIndex).UpperRight) {
            Trapezoids.at(upperRight.value()).UpperLeft = maybet4Index.value_or(t1Index);
        }

        if (auto lowerRight = Trapezoids.at(trapezoidIndex).LowerRight) {
            Trapezoids.at(lowerRight.value()).LowerLeft = maybet4Index.value_or(t2Index);
        }

        if (auto upperLeft = Trapezoids.at(trapezoidIndex).UpperLeft) {
            Trapezoids.at(upperLeft.value()).UpperRight = maybet3Index.value_or(t1Index);
        }

        if (auto lowerLeft = Trapezoids.at(trapezoidIndex).LowerLeft) {
            Trapezoids.at(lowerLeft.value()).LowerRight = maybet3Index.value_or(t2Index);
        }

        Trapezoids.at(trapezoidIndex) = t1;
        Trapezoids.push_back(t2);
        if (maybet3.has_value()) {
            Trapezoids.push_back(maybet3.value());
        }
        if (maybet4.has_value()) {
            Trapezoids.push_back(maybet4.value());
        }

        return SplitResult{{std::pair{t1Index, t2Index}}, maybet3Index, maybet4Index};
    }

    SplitResult
    TrapezoidalMap::Split(const LineSegment2D& segment, const std::vector<trapezoid_index>& trapezoidIndices) {
        COMPG_ASSERT(!trapezoidIndices.empty(), "Expected at least one trapezoid");
        if (trapezoidIndices.size() == 1) {
            return Split(segment, trapezoidIndices.at(0));
        }
        const auto [v0, v1] = GetEndPoints(segment);

        auto topLeftPoint = v0;
        std::optional<trapezoid_index> topUpperLeft{};

        auto bottomLeftPoint = v0;
        std::optional<trapezoid_index> bottomLowerLeft{};

        std::vector<TrapezoidRecord> topTraps;
        std::vector<TrapezoidRecord> bottomTraps;
        std::vector<std::tuple<std::size_t, std::size_t>> neighbors;
        for (auto [i, trapezoidIndex] :
             trapezoidIndices | std::views::take(trapezoidIndices.size() - 1) | std::views::enumerate) {
            neighbors.emplace_back(topTraps.size(), bottomTraps.size());

            const auto trapezoid = Trapezoids.at(trapezoidIndex);
            const auto side = FindSide(v0, v1, trapezoid.RightPoint);
            if (side == PointSide::Negative) {
                topTraps.emplace_back(
                    topLeftPoint, trapezoid.RightPoint, trapezoid.TopSegment, segment, topUpperLeft, std::nullopt,
                    trapezoid.UpperRight, std::nullopt
                );
                topLeftPoint = trapezoid.RightPoint;
                topUpperLeft = Trapezoids.at(trapezoidIndices.at(i + 1)).UpperLeft;

            } else if (side == PointSide::Positive) {
                bottomTraps.emplace_back(
                    bottomLeftPoint, trapezoid.RightPoint, segment, trapezoid.BottomSegment, std::nullopt,
                    bottomLowerLeft, std::nullopt, trapezoid.LowerRight
                );
                bottomLeftPoint = trapezoid.RightPoint;
                bottomLowerLeft = Trapezoids.at(trapezoidIndices.at(i + 1)).LowerLeft;

            } else {
                COMPG_THROW("Point is collinear");
            }
        }

        neighbors.emplace_back(topTraps.size(), bottomTraps.size());
        const auto lastTrapezoid = Trapezoids.at(trapezoidIndices.back());
        topTraps.emplace_back(
            topLeftPoint, v1, lastTrapezoid.TopSegment, segment, topUpperLeft, std::nullopt, lastTrapezoid.UpperRight,
            std::nullopt
        );
        bottomTraps.emplace_back(
            bottomLeftPoint, v1, segment, lastTrapezoid.BottomSegment, std::nullopt, lastTrapezoid.LowerLeft,
            std::nullopt, lastTrapezoid.LowerRight
        );

        std::optional<TrapezoidRecord> maybet1{};
        if (!AreEqual(Trapezoids.at(trapezoidIndices.at(0)).LeftPoint, v0)) {
            maybet1 = Trapezoids.at(trapezoidIndices.at(0));
            maybet1->RightPoint = v0;
            maybet1->UpperRight = std::nullopt;
            maybet1->LowerRight = std::nullopt;
        }

        std::optional<TrapezoidRecord> maybet2{};
        if (!AreEqual(Trapezoids.at(trapezoidIndices.back()).RightPoint, v1)) {
            maybet2 = Trapezoids.at(trapezoidIndices.back());
            maybet2->LeftPoint = v1;
            maybet2->UpperLeft = std::nullopt;
            maybet2->LowerLeft = std::nullopt;
        }

        const std::size_t numNewTraps = topTraps.size() + bottomTraps.size();
        std::vector<trapezoid_index> newTrapsIndices(numNewTraps);
        auto insertIt = newTrapsIndices.begin();

        std::optional<trapezoid_index> maybet1Index{};
        if (maybet1.has_value()) {
            maybet1Index = trapezoidIndices.at(0);
        } else {
            *insertIt = trapezoidIndices.at(0);
            ++insertIt;
        }
        std::optional<trapezoid_index> maybet2Index{};
        if (maybet2.has_value()) {
            maybet2Index = trapezoidIndices.back();
        } else {
            *insertIt = trapezoidIndices.back();
            ++insertIt;
        }

        insertIt = std::copy(trapezoidIndices.begin() + 1, trapezoidIndices.end() - 1, insertIt);
        std::iota(insertIt, newTrapsIndices.end(), Trapezoids.size());

        std::vector<trapezoid_index> topIndices(newTrapsIndices.begin(), newTrapsIndices.begin() + topTraps.size());
        std::vector<trapezoid_index> bottomIndices(newTrapsIndices.begin() + topTraps.size(), newTrapsIndices.end());

        for (const auto [i, trapezoidIndex] :
             topIndices | std::views::take(topIndices.size() - 1) | std::views::enumerate) {
            topTraps.at(i).LowerRight = topIndices.at(i + 1);
            topTraps.at(i + 1).LowerLeft = trapezoidIndex;
        }

        for (const auto [i, trapezoidIndex] :
             bottomIndices | std::views::take(bottomIndices.size() - 1) | std::views::enumerate) {
            bottomTraps.at(i).UpperRight = bottomIndices.at(i + 1);
            bottomTraps.at(i + 1).UpperLeft = trapezoidIndex;
        }

        if (maybet1.has_value()) {
            topTraps.at(0).UpperLeft = maybet1Index.value();
            maybet1->UpperRight = topIndices.at(0);

            bottomTraps.at(0).LowerLeft = maybet1Index.value();
            maybet1->LowerRight = bottomIndices.at(0);
        } else {
            const auto upperLeft = Trapezoids.at(trapezoidIndices.at(0)).UpperLeft;
            topTraps.at(0).UpperLeft = upperLeft;
            if (upperLeft.has_value()) {
                Trapezoids.at(upperLeft.value()).UpperRight = topIndices.at(0);
            }

            const auto lowerLeft = Trapezoids.at(trapezoidIndices.at(0)).LowerLeft;
            bottomTraps.at(0).LowerLeft = lowerLeft;
            if (lowerLeft.has_value()) {
                Trapezoids.at(lowerLeft.value()).LowerRight = bottomIndices.at(0);
            }
        }

        if (maybet2.has_value()) {
            topTraps.back().UpperRight = maybet2Index.value();
            maybet2->UpperLeft = topIndices.back();

            bottomTraps.back().LowerRight = maybet2Index.value();
            maybet2->LowerLeft = bottomIndices.back();
        } else {
            const auto upperRight = Trapezoids.at(trapezoidIndices.back()).UpperRight;
            topTraps.back().UpperRight = upperRight;
            if (upperRight.has_value()) {
                Trapezoids.at(upperRight.value()).UpperLeft = topIndices.back();
            }

            const auto lowerRight = Trapezoids.at(trapezoidIndices.back()).LowerRight;
            bottomTraps.back().LowerRight = lowerRight;
            if (lowerRight.has_value()) {
                Trapezoids.at(lowerRight.value()).LowerLeft = bottomIndices.back();
            }
        }

        for (const auto& [i, trapezoid] : topTraps | std::views::enumerate) {
            const auto trapezoidIndex = topIndices.at(i);
            if (const auto upperLeft = trapezoid.UpperLeft) {
                Trapezoids.at(upperLeft.value()).UpperRight = trapezoidIndex;
            }
            if (const auto upperRight = trapezoid.UpperRight) {
                Trapezoids.at(upperRight.value()).UpperLeft = trapezoidIndex;
            }
        }

        for (const auto& [i, trapezoid] : bottomTraps | std::views::enumerate) {
            const auto trapezoidIndex = bottomIndices.at(i);
            if (const auto lowerLeft = trapezoid.LowerLeft) {
                Trapezoids.at(lowerLeft.value()).LowerRight = trapezoidIndex;
            }
            if (const auto lowerRight = trapezoid.LowerRight) {
                Trapezoids.at(lowerRight.value()).LowerLeft = trapezoidIndex;
            }
        }

        if (maybet1.has_value()) {
            Trapezoids.at(maybet1Index.value()) = maybet1.value();
        }

        if (maybet2.has_value()) {
            Trapezoids.at(maybet2Index.value()) = maybet2.value();
        }

        for (const auto& [i, trapezoid] : topTraps | std::views::enumerate) {
            const auto trapezoidIndex = topIndices.at(i);
            if (trapezoidIndex < Trapezoids.size()) {
                Trapezoids.at(trapezoidIndex) = trapezoid;
            } else {
                COMPG_ASSERT(trapezoidIndex == Trapezoids.size(), "oops");
                Trapezoids.push_back(trapezoid);
            }
        }

        for (const auto& [i, trapezoid] : bottomTraps | std::views::enumerate) {
            const auto trapezoidIndex = bottomIndices.at(i);
            if (trapezoidIndex < Trapezoids.size()) {
                Trapezoids.at(trapezoidIndex) = trapezoid;
            } else {
                COMPG_ASSERT(trapezoidIndex == Trapezoids.size(), "oops");
                Trapezoids.push_back(trapezoid);
            }
        }

        return {
            .TrapezoidIndices = neighbors | std::views::transform([&topIndices, bottomIndices](const auto& tup) {
                                    const auto [ti, bi] = tup;
                                    return std::tuple{topIndices.at(ti), bottomIndices.at(bi)};
                                })
                                | std::ranges::to<std::vector>(),
            .LeftTrapezoidIndex = maybet1Index,
            .RightTrapezoidIndex = maybet2Index
        };
    }

} // namespace compg