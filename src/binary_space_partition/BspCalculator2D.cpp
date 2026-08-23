#include "binary_space_partition/BspCalculator2D.hpp"

#include "binary_space_partition/BspLeafDfs.hpp"
#include "binary_space_partition/LowDensityPartition.hpp"
#include "data_structures/Cropper.hpp"
#include "math/BoundingBox.hpp"
#include "math/Conversions.hpp"
#include "math/Geometry.hpp"
#include "math/Math.hpp"
#include "math/primitives/Hyperplane.hpp"

namespace compg {
    namespace details {
        std::vector<LineSegment2D> FindIntersections(
            const HalfPlane<Hyperplane2D>& halfPlane, const std::vector<LineSegment2D>& segments, Float epsilon
        ) {
            std::vector<LineSegment2D> result;

            for (const auto& segment : segments) {
                Cropper2D::Crop(segment, halfPlane).transform([&halfPlane, epsilon, &result](const auto& intersection) {
                    if (FindSide<2UZ>(halfPlane, intersection[0], epsilon) != PointSide::Collinear
                        || FindSide<2UZ>(halfPlane, intersection[1], epsilon) != PointSide::Collinear) {

                        result.push_back(intersection);
                    }
                    return intersection;
                });
            }

            return result;
        }

        std::vector<LineSegment2D> FindCollinearSegments(
            const HalfPlane<Hyperplane2D>& halfPlane, const std::vector<LineSegment2D>& segments, Float epsilon
        ) {
            std::vector<LineSegment2D> result;
            for (const auto& segment : segments) {
                if (FindSide<2UZ>(halfPlane, segment[0], epsilon) == PointSide::Collinear
                    && FindSide<2UZ>(halfPlane, segment[1], epsilon) == PointSide::Collinear) {
                    result.push_back(segment);
                }
            }
            return result;
        }

        std::unique_ptr<BspTreeNode2D> FindPartition(const std::vector<LineSegment2D>& segments, Float epsilon) {
            if (segments.size() <= 1) {
                return std::make_unique<BspTreeNode2D>(BspTreeLeafNode2D{
                    segments | std::views::transform([](const auto& s) { return ElementaryObject<2UZ>{s}; })
                    | std::ranges::to<std::vector>()
                });
            }

            const Line2D split{segments.front()[0], segments.front()[1]};
            const auto hyperplane = ConvertTo<Hyperplane2D>(split);
            HalfPlane<Hyperplane2D> halfPlane{hyperplane};

            const std::vector<LineSegment2D> positiveSegments = FindIntersections(halfPlane, segments, epsilon);
            halfPlane.Flip();
            const std::vector<LineSegment2D> negativeSegments = FindIntersections(halfPlane, segments, epsilon);
            std::vector<LineSegment2D> collinearSegments = FindCollinearSegments(halfPlane, segments, epsilon);

            auto negativeChild = FindPartition(negativeSegments, epsilon);
            auto positiveChild = FindPartition(positiveSegments, epsilon);

            return std::make_unique<BspTreeNode2D>(BspTreeInternalNode2D{
                hyperplane, std::move(collinearSegments) | std::views::transform([](const auto& s) {
                                return ElementaryObject<2UZ>{s};
                            }) | std::ranges::to<std::vector>(),
                std::move(negativeChild), std::move(positiveChild)
            });
        }

        std::vector<Vertex2D> FindGuards(const std::vector<LineSegment2D>& segments) {
            std::vector<Vertex2D> guards;
            guards.reserve(segments.size() * 4);

            for (const auto& segment : segments) {
                const Float xMin = std::min(segment[0][0], segment[1][0]);
                const Float xMax = std::max(segment[0][0], segment[1][0]);
                const Float yMin = std::min(segment[0][1], segment[1][1]);
                const Float yMax = std::max(segment[0][1], segment[1][1]);

                guards.emplace_back(xMin, yMin);
                guards.emplace_back(xMin, yMax);
                guards.emplace_back(xMax, yMin);
                guards.emplace_back(xMax, yMax);
            }

            return guards;
        }

        bool HasDenseLeaf(
            const std::unique_ptr<BspTreeNode2D>& root, const std::vector<LineSegment2D>& segments,
            std::size_t guardThreshold, Float epsilon
        ) {
            auto isDense
                = [&segments, NumMaxFragments = 5 * guardThreshold, epsilon](
                      const std::vector<HalfPlane<Hyperplane2D>>& halfPlanes, const std::unique_ptr<BspTreeNode2D>&
                  ) {
                      std::size_t numFragments{};
                      for (std::size_t i = 0; i < segments.size() && numFragments <= NumMaxFragments; ++i) {
                          Cropper2D::Crop(segments.at(i), halfPlanes)
                              .transform([&numFragments, epsilon](const LineSegment2D& fragment) {
                                  numFragments += AreEqual(fragment[0], fragment[1], epsilon) ? 0 : 1;
                                  return fragment;
                              });
                      }

                      // early exit
                      return numFragments > NumMaxFragments;
                  };
            BspLeafDfs<2UZ, decltype(isDense)> dfs{.Function = isDense};
            return dfs.Call(root);
        }
    } // namespace details

    BspTree2D BspCalculator2D::FindPartition(const std::vector<LineSegment2D>& segments) const {
        auto segmentsPermutation{segments};
        RandomPermutation(segmentsPermutation, Seed);
        auto root = details::FindPartition(segmentsPermutation, Epsilon);
        return BspTree2D{std::move(root)};
    }

    BspTree2D BspCalculator2D::FindLowDensityPartition(const std::vector<LineSegment2D>& segments) const {
        auto guards = details::FindGuards(segments);
        std::size_t guardThreshold{1};
        bool done = false;
        const Box2D box{Pad(FindBoundingBox(segments), 1, 1)};
        std::unique_ptr<BspTreeNode2D> root{nullptr};

        while (!done) {
            guardThreshold *= 2;
            root = Phase1<2UZ>(box, guards, guardThreshold, Epsilon);
            done = !details::HasDenseLeaf(root, segments, guardThreshold, Epsilon);
        }

        SetLeaves(segments, root);
        return {std::move(root)};
    }

    void
    BspCalculator2D::SetLeaves(const std::vector<LineSegment2D>& segments, std::unique_ptr<BspTreeNode2D>& root) const {
        auto updateLeaf
            = [&segments,
               this](const std::vector<HalfPlane<Hyperplane2D>>& halfPlanes, std::unique_ptr<BspTreeNode2D>& node) {
                  std::vector<LineSegment2D> fragments;
                  for (const LineSegment2D& segment : segments) {
                      Cropper2D::Crop(segment, halfPlanes).transform([this, &fragments](const LineSegment2D& fragment) {
                          if (!AreEqual(fragment[0], fragment[1], Epsilon)) {
                              fragments.push_back(fragment);
                          }
                          return fragment;
                      });
                  }

                  RandomPermutation(fragments, Seed);
                  node = details::FindPartition(fragments, Epsilon);
                  return false;
              };

        BspLeafDfs<2UZ, decltype(updateLeaf)> dfs{.Function = updateLeaf};
        dfs.Call(root);
    }
} // namespace compg
