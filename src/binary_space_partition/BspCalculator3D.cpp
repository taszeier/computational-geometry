#include "binary_space_partition/BspCalculator3D.hpp"

#include "binary_space_partition/BspLeafDfs.hpp"
#include "binary_space_partition/LowDensityPartition.hpp"
#include "delaunay/IndexTriangle.hpp"
#include "math/BoundingBox.hpp"
#include "math/Conversions.hpp"
#include "math/Geometry.hpp"

namespace compg {
    namespace details {
        std::vector<Vertex3D> Crop(const HalfPlane<Hyperplane3D>& halfPlane, const auto& vertices, Float epsilon) {
            if (vertices.empty()) {
                return {};
            }

            const auto sides
                = std::views::transform(
                      vertices, [&halfPlane, epsilon](const auto& v) { return FindSide<3UZ>(halfPlane, v, epsilon); }
                  )
                  | std::ranges::to<std::vector>();
            std::vector<Vertex3D> result;
            auto next = [&vertices](std::size_t i) { return (i + 1) % vertices.size(); };
            auto prev = [&vertices](std::size_t i) { return (i + (vertices.size() - 1)) % vertices.size(); };

            if (const auto posIt = std::ranges::find(sides, PointSide::Positive); posIt != sides.end()) {
                const std::size_t start_i = std::distance(sides.begin(), posIt);

                result.push_back(vertices.at(start_i));

                std::size_t i = next(start_i);
                while (i != start_i) {
                    while (i != start_i && sides.at(i) != PointSide::Negative) {
                        result.push_back(vertices.at(i));
                        i = next(i);
                    }
                    if (i == start_i) {
                        break;
                    }

                    std::size_t prev_i = prev(i);
                    const auto exitIntersection
                        = FindIntersection(halfPlane.Plane, vertices.at(prev_i), vertices.at(i), epsilon);
                    COMPG_ASSERT(exitIntersection.has_value(), "Expected the line segment to intersect the hyperplane");
                    result.push_back(exitIntersection.value());

                    while (sides.at(i) == PointSide::Negative) {
                        i = next(i);
                    }

                    prev_i = prev(i);
                    const auto entryIntersection
                        = FindIntersection(halfPlane.Plane, vertices.at(prev_i), vertices.at(i), epsilon);
                    COMPG_ASSERT(
                        entryIntersection.has_value(), "Expected the line segment to intersect the hyperplane"
                    );
                    result.push_back(entryIntersection.value());
                }

                std::size_t j = 0;
                std::size_t insert_pos = 0;
                while (j < result.size()) {
                    result.at(insert_pos) = result.at(j);
                    ++j;

                    while (j < result.size() && AreEqual(result.at(insert_pos), result.at(j), epsilon)) {
                        ++j;
                    }

                    ++insert_pos;
                }
                result.erase(std::next(result.begin(), insert_pos), result.end());
            }
            if (result.size() < 3) {
                result.clear();
            }
            return result;
        }

        std::vector<Vertex3D>
        Crop(const std::vector<HalfPlane<Hyperplane3D>>& halfPlanes, const auto& vertices, Float epsilon) {
            std::vector<Vertex3D> result{vertices.begin(), vertices.end()};

            for (std::size_t i = 0; i < halfPlanes.size() && !result.empty(); ++i) {
                result = Crop(halfPlanes.at(i), result, epsilon);
            }
            return result;
        }

        std::vector<Triangle3D> FindIntersections(
            const HalfPlane<Hyperplane3D>& halfPlane, const std::vector<Triangle3D>& triangles, Float epsilon
        ) {
            std::vector<Triangle3D> result;

            for (const auto& triangle : triangles) {
                const auto vertices = Crop(halfPlane, triangle.GetVertices(), epsilon);
                for (std::size_t k = 2; k < vertices.size(); ++k) {
                    result.emplace_back(vertices.at(0), vertices.at(k - 1), vertices.at(k));
                }
            }
            return result;
        }

        std::vector<Triangle3D> FindCollinearSegments(
            const HalfPlane<Hyperplane3D>& halfPlane, const std::vector<Triangle3D>& triangles, Float epsilon
        ) {
            std::vector<Triangle3D> collinear;
            for (const auto& triangle : triangles) {
                if (std::ranges::all_of(triangle.GetVertices(), [&halfPlane, epsilon](const auto& v) {
                        return FindSide<3UZ>(halfPlane, v, epsilon) == PointSide::Collinear;
                    })) {
                    collinear.push_back(triangle);
                }
            }

            return collinear;
        }

        std::unique_ptr<BspTreeNode3D> FindPartition(const std::vector<Triangle3D>& triangles, Float epsilon) {
            if (triangles.size() <= 1) {
                return std::make_unique<BspTreeNode3D>(BspTreeLeafNode3D{
                    triangles | std::views::transform([](const auto& t) { return ElementaryObject3D{t}; })
                    | std::ranges::to<std::vector>()
                });
            }

            const auto hyperplane = ConvertTo<Hyperplane3D>(triangles.front());
            HalfPlane<Hyperplane3D> halfPlane{hyperplane};

            const std::vector<Triangle3D> positiveFragments = FindIntersections(halfPlane, triangles, epsilon);
            halfPlane.Flip();
            const std::vector<Triangle3D> negativeFragments = FindIntersections(halfPlane, triangles, epsilon);
            std::vector<Triangle3D> collinearTriangles = FindCollinearSegments(halfPlane, triangles, epsilon);

            auto negativeChild = FindPartition(negativeFragments, epsilon);
            auto positiveChild = FindPartition(positiveFragments, epsilon);

            return std::make_unique<BspTreeNode3D>(BspTreeInternalNode3D{
                hyperplane, std::move(collinearTriangles) | std::views::transform([](const auto& s) {
                                return ElementaryObject3D{s};
                            }) | std::ranges::to<std::vector>(),
                std::move(negativeChild), std::move(positiveChild)
            });
        }

        std::vector<Vertex3D> FindGuards(const std::vector<Triangle3D>& triangles) {
            std::vector<Vertex3D> guards;
            guards.reserve(triangles.size() * 8);

            for (const auto& triangle : triangles) {
                const auto boundingBox = FindBoundingBox(triangle);
                std::ranges::copy(boundingBox.GetCorners(), std::back_inserter(guards));
            }

            return guards;
        }

        bool HasDenseLeaf(
            const std::unique_ptr<BspTreeNode3D>& root, const std::vector<Triangle3D>& triangles,
            std::size_t guardThreshold, Float epsilon
        ) {
            auto isDense
                = [&triangles, NumMaxFragments = 5 * guardThreshold, epsilon](
                      const std::vector<HalfPlane<Hyperplane3D>>& halfPlanes, const std::unique_ptr<BspTreeNode3D>&
                  ) {
                      std::size_t numFragments{};
                      for (std::size_t i = 0; i < triangles.size() && numFragments <= NumMaxFragments; ++i) {
                          const auto fragment = Crop(halfPlanes, triangles.at(i).GetVertices(), epsilon);
                          numFragments += fragment.empty() ? 0 : 1;
                      }

                      // early exit
                      return numFragments > NumMaxFragments;
                  };
            BspLeafDfs<3UZ, decltype(isDense)> dfs{isDense};
            return dfs.Call(root);
        }

    } // namespace details

    BspTree3D BspCalculator3D::FindPartition(const std::vector<Triangle3D>& triangles) const {
        auto trianglesPermutation{triangles};
        RandomPermutation(trianglesPermutation, Seed);
        auto root = details::FindPartition(trianglesPermutation, Epsilon);
        return BspTree3D{std::move(root)};
    }

    BspTree3D BspCalculator3D::FindLowDensityPartition(const std::vector<Triangle3D>& triangles) const {
        auto guards = details::FindGuards(triangles);
        std::size_t guardThreshold{1};
        bool done = false;
        const Box3D box{Pad(FindBoundingBox(triangles), 1, 1, 1)};
        std::unique_ptr<BspTreeNode3D> root{nullptr};

        while (!done) {
            guardThreshold *= 2;
            root = Phase1<3UZ>(box, guards, guardThreshold, Epsilon);
            done = !details::HasDenseLeaf(root, triangles, guardThreshold, Epsilon);
        }

        SetLeaves(triangles, root);
        return {std::move(root)};
    }

    void
    BspCalculator3D::SetLeaves(const std::vector<Triangle3D>& triangles, std::unique_ptr<BspTreeNode3D>& root) const {
        auto updateLeaf
            = [&triangles,
               this](const std::vector<HalfPlane<Hyperplane3D>>& halfPlanes, std::unique_ptr<BspTreeNode3D>& node) {
                  std::vector<Triangle3D> fragments;
                  for (const auto& triangle : triangles) {
                      const auto vertices = details::Crop(halfPlanes, triangle.GetVertices(), Epsilon);
                      for (std::size_t i = 2; i < vertices.size(); ++i) {
                          fragments.emplace_back(vertices.at(0), vertices.at(i - 1), vertices.at(i));
                      }
                  }

                  RandomPermutation(fragments, Seed);
                  node = details::FindPartition(fragments, Epsilon);
                  return false;
              };

        BspLeafDfs<3UZ, decltype(updateLeaf)> dfs{updateLeaf};
        dfs.Call(root);
    }
} // namespace compg
