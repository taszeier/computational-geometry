#pragma once

#include "math/Geometry.hpp"
#include "math/Math.hpp"

namespace compg {
    template <std::size_t K>
    Vertex<K> FindPartitionCenter(
        const Box<K>& box, std::size_t guardThreshold, const auto& regionSize, std::vector<Vertex<K>>& guards
    ) {
        Vertex<K> center = box.GetCenter();
        const auto corners = box.GetCorners();
        const auto isDenseQuadrant = [guardThreshold](auto size) { return size > guardThreshold; };
        const auto shrinkIt = std::ranges::find_if(regionSize, isDenseQuadrant);
        if (shrinkIt != regionSize.end()
            && std::find_if(std::next(shrinkIt), regionSize.end(), isDenseQuadrant) == regionSize.end()) {
            // shrink
            const std::size_t numNotInInterior = guards.size() - *shrinkIt;
            if (numNotInInterior < guardThreshold) {
                // actually shrink
                const std::size_t shrinkIndex = std::distance(regionSize.begin(), shrinkIt);
                const Vertex<K>& shrinkCorner{corners.at(shrinkIndex)};
                const auto shrinkBoundaryIt = std::next(guards.begin(), guards.size() - guardThreshold);
                std::ranges::nth_element(guards, shrinkBoundaryIt, Less{}, [&shrinkCorner](const Vertex<K>& guard) {
                    return math::InfinityNorm<K>(shrinkCorner - guard);
                });

                const Float shrinkCornerLength = math::InfinityNorm<K>(shrinkCorner - *shrinkBoundaryIt);

                Vertex<K> splitCenter;
                for (const auto i : Indices<K>) {
                    splitCenter[i] = shrinkCorner[i] + (center[i] > shrinkCorner[i] ? 1 : -1) * shrinkCornerLength;
                }
                return splitCenter;
            }
            return center;
        }
        return center;
    }

    template <std::size_t K>
    void PartitionGuards(
        const Box<K>& box, const std::vector<Vertex<K>>& guards, const auto& regions, const Vertex<K>& partitionCenter,
        Float epsilon, std::vector<std::vector<Vertex<K>>>& regionGuards
    ) {
        auto interiorGuards = std::views::filter(guards, [&partitionCenter, epsilon](const Vertex<K>& guard) {
            return (partitionCenter - guard).cwiseAbs().minCoeff() > epsilon;
        });

        const auto corners = box.GetCorners();

        for (const auto& guard : interiorGuards) {
            const auto regionIt
                = std::ranges::find_if(regions, [&guard](const Box<K>& region) { return region.Contains(guard); });
            COMPG_ASSERT(regionIt != regions.end(), "Expected at least one region to contain the guard");
            const std::size_t regionIndex = std::distance(regions.begin(), regionIt);
            regionGuards.at(regionIndex).push_back(guard);
        }
    }

    template <std::size_t K>
    std::unique_ptr<BspTreeNode<K>>
    Phase1(const Box<K>& box, std::vector<Vertex<K>>& guards, std::size_t guardThreshold, Float epsilon);

    template <std::size_t K>
    struct Phase1Dfs {
        std::unique_ptr<BspTreeNode<K>> operator()(std::size_t depth) {
            if (depth == K) {
                return Phase1<K>(Regions.at(RegionIndex), RegionGuards.at(RegionIndex), GuardThreshold, Epsilon);
                ++RegionIndex;
            }
            BspTreeInternalNode<K> node{Splits.at(depth)};
            node.NegativeChild = operator()(depth + 1);
            node.PositiveChild = operator()(depth + 1);

            return std::make_unique<BspTreeNode<K>>(std::move(node));
        };

        const std::vector<Hyperplane<K>>& Splits;
        const std::vector<Box<K>>& Regions;
        std::vector<std::vector<Vertex<K>>>& RegionGuards;
        Vertex<K> PartitionCenter;
        std::size_t GuardThreshold;
        Float Epsilon;

        std::size_t RegionIndex{};
    };

    template <std::size_t K>
    std::unique_ptr<BspTreeNode<K>>
    Phase1(const Box<K>& box, std::vector<Vertex<K>>& guards, std::size_t guardThreshold, Float epsilon) {
        if (guards.size() <= guardThreshold) {
            return std::make_unique<BspTreeNode<K>>(BspTreeLeafNode<K>{});
        }

        const auto corners = box.GetCorners();
        std::vector<std::size_t> regionSize(corners.size());

        const auto center = box.GetCenter();
        auto interiorGuards = std::views::filter(guards, [&center, epsilon](const Vertex<K>& guard) {
            return (center - guard).cwiseAbs().minCoeff() > epsilon;
        });

        const auto initialRegions
            = std::views::transform(corners, [&center](const auto& corner) { return Box<K>{center, corner}; })
              | std::ranges::to<std::vector>();

        for (const auto& guard : interiorGuards) {
            const auto regionIt = std::ranges::find_if(initialRegions, [&guard](const Box<K>& region) {
                return region.Contains(guard);
            });
            COMPG_ASSERT(regionIt != initialRegions.end(), "Expected at least one region to contain the guard");
            const std::size_t regionIndex = std::distance(initialRegions.begin(), regionIt);
            ++regionSize.at(regionIndex);
        }

        const Vertex<K> partitionCenter = FindPartitionCenter<K>(box, guardThreshold, regionSize, guards);
        const auto regions
            = std::views::transform(
                  corners, [&partitionCenter](const auto& corner) { return Box<K>{partitionCenter, corner}; }
              )
              | std::ranges::to<std::vector>();
        std::vector<std::vector<Vertex<K>>> regionGuards(regions.size());
        PartitionGuards<K>(box, guards, regions, partitionCenter, epsilon, regionGuards);

        const std::vector<Hyperplane<K>> splits
            = std::views::transform(
                  Indices<K>,
                  [&partitionCenter](std::size_t i) {
                      return ConvertTo<Hyperplane<K>>(AxesAlignedHyperplane<K>{i, partitionCenter[i]});
                  }
              )
              | std::ranges::to<std::vector>();

        Phase1Dfs<K> dfs{splits, regions, regionGuards, partitionCenter, guardThreshold, epsilon};
        return dfs(0UZ);
    }
} // namespace compg