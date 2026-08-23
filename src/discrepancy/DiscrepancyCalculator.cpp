#include "discrepancy/DiscrepancyCalculator.hpp"

#include "common/Common.hpp"
#include "data_structures/DisjointSetUnion.hpp"
#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include "discrepancy/ArrangementCalculator.hpp"
#include "math/Conversions.hpp"
#include "math/Geometry.hpp"
#include "math/Math.hpp"

namespace compg {
    namespace details {
        const Box2D UnitBox{{0, 0}, {1, 1}};
        const std::array<Vertex2D, 4> UnitBoxCorners{{{0, 0}, {1, 0}, {1, 1}, {0, 1}}};
        const std::array UnitBoxSides{
            LineSegment2D{UnitBoxCorners.at(0), UnitBoxCorners.at(1)},
            LineSegment2D{UnitBoxCorners.at(1), UnitBoxCorners.at(2)},
            LineSegment2D{UnitBoxCorners.at(2), UnitBoxCorners.at(3)},
            LineSegment2D{UnitBoxCorners.at(3), UnitBoxCorners.at(0)}
        };

        Line2D DualLine(const Vertex2D& v) {
            const Vertex2D v0{0, -v[1]};
            const Vertex2D v1{1, v[0] - v[1]};
            return {v0, v1};
        }

        Float FindContinuousMeasure(const Line2D& line, PointSide side = PointSide::Positive) {
            COMPG_ASSERT(side != PointSide::Collinear, "Expected a valid side");

            const auto intersections
                = UnitBoxSides
                  | std::views::transform([&line](const auto& side) { return FindIntersection(line, side); })
                  | std::ranges::to<std::vector>();
            const auto it1 = std::ranges::find_if(intersections, &std::optional<Vertex2D>::has_value);
            COMPG_ASSERT(it1 != intersections.end(), "Expected the line to intersect the unit box");
            const auto it2 = std::find_if(std::next(it1), intersections.end(), [it1](const auto& intersection) {
                return intersection.has_value() && !AreEqual(it1->value(), intersection.value());
            });
            COMPG_ASSERT(it2 != intersections.end(), "Expected the line to intersect the unit box twice");

            std::vector<Vertex2D> corners{it1->value(), it2->value()};
            std::ranges::copy(
                UnitBoxCorners
                    | std::views::filter([&line, side](const auto& corner) { return FindSide(line, corner) == side; }),
                std::back_inserter(corners)
            );

            return FindConvexPolygonAreaUnordered(corners);
        }

        auto CountSides(const std::vector<Vertex2D>& vertices, const Line2D& line) {
            std::array<std::size_t, 3> counts{};
            std::ranges::for_each(vertices, [&](const auto& v) {
                const auto side = FindSide(line, v);
                if (side == PointSide::Positive) {
                    ++counts[0];
                } else if (side == PointSide::Collinear) {
                    ++counts[1];
                } else {
                    COMPG_ASSERT(side == PointSide::Negative, "Oops");
                    ++counts[2];
                }
            });

            return counts;
        }

        auto FindDiscrepancies(const Line2D& line, const std::vector<Vertex2D>& vertices) {
            const auto continuousMeasure1 = FindContinuousMeasure(line);
            const auto continuousMeasure2 = 1 - continuousMeasure1;
            const auto [positive, collinear, negative] = CountSides(vertices, line);
            const Float discreteMeasure1
                = static_cast<Float>(positive + collinear) / static_cast<Float>(vertices.size());
            const Float discreteMeasure2
                = static_cast<Float>(negative + collinear) / static_cast<Float>(vertices.size());

            const auto d1 = std::abs(discreteMeasure1 - continuousMeasure1);
            const auto d2 = std::abs(discreteMeasure2 - continuousMeasure2);
            return std::tuple{d1, d2};
        }

        Float FindDiscrepancyType1(const std::vector<Vertex2D>& vertices) {
            Float result{};
            std::ranges::for_each(
                std::views::cartesian_product(vertices, UnitBoxCorners), [&result, &vertices](const auto& tup) {
                    const auto& [vertex, corner] = tup;
                    if (AreEqual(vertex, corner)) {
                        return;
                    }
                    const Line2D line{vertex, corner};
                    const auto [d1, d2] = FindDiscrepancies(line, vertices);
                    result = std::max(result, std::max(d1, d2));
                }
            );

            return result;
        }

        std::size_t CountLinesAbove(const Vertex2D& vertex, auto&& lines, Float epsilon = EPSILON) {
            const auto verticalLine = MakeLine2DFromX(vertex[0]);
            return std::ranges::count_if(lines, [&verticalLine, &vertex, epsilon](const auto& line) {
                const auto intersection = FindIntersection(line, verticalLine);
                COMPG_ASSERT(intersection.has_value(), "Expected a non-vertical line");
                return intersection.value()[1] > vertex[1] + epsilon;
            });
        }

        using counter_type = std::unordered_map<DoublyConnectedEdgeList::vertex_index, std::size_t>;

        Float FindArrangementDiscrepancy(
            const DoublyConnectedEdgeList& arrangement, const counter_type& numLinesAboveMap,
            const counter_type& numLinesOnMap, const auto& vertices
        ) {
            Float result{};
            const auto numDualLines = vertices.size();

            for (const auto& [vertexIndex, numLinesAbove] : numLinesAboveMap) {
                const auto numLinesOn = numLinesOnMap.at(vertexIndex);
                COMPG_ASSERT(numLinesAbove + numLinesOn <= numDualLines, "This should be impossible");
                const auto numLinesBelow = numDualLines - numLinesOn - numLinesAbove;
                const auto line = DualLine(arrangement.GetVertex(vertexIndex).Vertex);

                const Float discreteMeasureBelow
                    = static_cast<Float>(numLinesAbove + numLinesOn) / static_cast<Float>(numDualLines);
                const Float continuousMeasureBelow = FindContinuousMeasure(line);
                const Float discreteMeasureAbove
                    = static_cast<Float>(numLinesBelow + numLinesOn) / static_cast<Float>(numDualLines);
                const Float continuousMeasureAbove = 1.0 - continuousMeasureBelow;

                result = std::max(result, std::abs(discreteMeasureBelow - continuousMeasureBelow));
                result = std::max(result, std::abs(discreteMeasureAbove - continuousMeasureAbove));
            }

            return result;
        }

        Float FindVerticalLinesDiscrepancy(const std::vector<Vertex2D>& vertices) {
            Float result{};
            DisjointSetUnion disjointSetUnion{vertices.size()};
            std::ranges::for_each(
                std::views::cartesian_product(vertices | std::views::enumerate, vertices | std::views::enumerate),
                [&disjointSetUnion](const auto& tup) {
                    const auto& [pair1, pair2] = tup;
                    const auto& [i, v1] = pair1;
                    const auto& [j, v2] = pair2;
                    if (i < j && math::IsZero(v1[0] - v2[0])) {
                        disjointSetUnion.Union(i, j);
                    }
                }
            );

            for (auto&& [rootIndex, equivalentIndices] : disjointSetUnion.FindDisjointSets()) {
                if (equivalentIndices.size() >= 2) {
                    const auto line = MakeLine2DFromX(vertices.at(rootIndex)[0]);
                    const auto [d1, d2] = FindDiscrepancies(line, vertices);
                    result = std::max(result, std::max(d1, d2));
                }
            }

            return result;
        }

        Float FindNonVerticalLinesDiscrepancy(const std::vector<Vertex2D>& vertices) {
            const auto duals = vertices | std::views::transform(DualLine) | std::ranges::to<std::vector>();
            const auto maybeArrangementDetails = Try(
                [](const auto& lines) {
                    const ArrangementCalculator calculator;
                    return calculator.FindArrangementDetails(lines);
                },
                duals
            );
            if (!maybeArrangementDetails) {
                return 0.0;
            }

            const auto [arrangement, firstEdgeMap] = maybeArrangementDetails.value();

            counter_type numLinesAbove;
            counter_type numLinesOn;
            std::ranges::for_each(
                duals | std::views::enumerate,
                [&duals, &arrangement, &firstEdgeMap, &numLinesAbove, &numLinesOn](const auto& tup) {
                    const auto& [index, line] = tup;
                    const auto firstEdge = firstEdgeMap.at(index);
                    const auto v0 = arrangement.GetVertex(arrangement.GetOriginIndex(firstEdge)).Vertex;
                    const auto level = CountLinesAbove(v0, duals);

                    WalkLine(
                        arrangement, firstEdge,
                        [firstEdge, &arrangement, previousEdge = firstEdge, level = level, &numLinesAbove,
                         &numLinesOn](const auto e) mutable {
                            if (e != firstEdge) {
                                auto f = arrangement.GetNextIndex(previousEdge);
                                std::size_t numAboveCrosses{};
                                std::size_t numBelowCrosses{};
                                const auto originIndex = arrangement.GetOriginIndex(e);
                                const auto origin = arrangement.GetVertex(originIndex).Vertex;
                                numLinesOn[originIndex] = 1;
                                while (f != e) {
                                    if (arrangement.GetVertex(arrangement.GetDestinationIndex(f)).Vertex[0]
                                        < origin[0]) {
                                        ++numAboveCrosses;
                                    } else {
                                        ++numBelowCrosses;
                                    }

                                    ++numLinesOn[originIndex];
                                    f = arrangement.GetNextIndex(arrangement.GetTwinIndex(f));
                                }

                                numLinesAbove[originIndex] = level - numAboveCrosses;
                                level -= numAboveCrosses;
                                level += numBelowCrosses;

                                previousEdge = e;
                            }
                        }
                    );
                }
            );

            return FindArrangementDiscrepancy(arrangement, numLinesAbove, numLinesOn, vertices);
        };

        Float FindDiscrepancyType2(const std::vector<Vertex2D>& vertices) {
            const auto d1 = FindNonVerticalLinesDiscrepancy(vertices);
            const auto d2 = FindVerticalLinesDiscrepancy(vertices);
            return std::max(d1, d2);
        }
    } // namespace details

    Float DiscrepancyCalculator::FindDiscrepancy(const std::vector<Vertex2D>& vertices) const {
        COMPG_ASSERT(!vertices.empty(), "Expected at least one vertex");
        std::ranges::for_each(vertices, [](const auto& vertex) {
            COMPG_ASSERT(details::UnitBox.Contains(vertex), "Expected the vertex to lie in the range [0,1] x [0,1]");
        });
        const auto d1 = details::FindDiscrepancyType1(vertices);
        const auto d2 = details::FindDiscrepancyType2(vertices);
        return std::max(d1, d2);
    }
} // namespace compg
