#include "quad_tree/CircuitQuadTree.hpp"
#include "math/Geometry.hpp"

#include <eigen3/Eigen/Dense>

#include "math/Conversions.hpp"

namespace compg {
    using UnsignedInteger = CircuitVertex::Scalar;

    namespace details {
        LineSegment2D AsFloatSegment(const CircuitSegment& segment) {
            return {{segment[0][0], segment[0][1]}, {segment[1][0], segment[1][1]}};
        }

        bool IntersectsBox(const CircuitSegment& circuitSegment, const Box2D& box) {
            const LineSegment2D segment = AsFloatSegment(circuitSegment);
            if (box.Contains(segment[0]) || box.Contains(segment[1])) {
                return true;
            }

            const auto v = GetCorners(box);
            for (const auto i : Indices<v.size()>) {
                const auto j = (i + 1) % v.size();
                if (FindIntersection(segment, LineSegment2D{v.at(i), v.at(j)}).has_value()) {
                    return true;
                }
            }

            return false;
        }

        bool IsUnitBox(const Box2D& box) {
            return static_cast<UnsignedInteger>(box.Size()) == 1;
        }

        bool IntersectsUnitBoxInterior(const CircuitSegment& segment, const Box2D& box) {
            COMPG_ASSERT(IsUnitBox(box), "Expected a unit box");
            const auto [xMin, xMax] = std::minmax(segment[0][0], segment[1][0]);
            const auto [yMin, yMax] = std::minmax(segment[0][1], segment[1][1]);
            return xMin <= box.GetLowerCorner()[0] && box.GetUpperCorner()[0] <= xMax && yMin <= box.GetLowerCorner()[1]
                   && box.GetUpperCorner()[1] <= yMax;
        }

        template <std::ranges::range SegmentRange>
            requires std::same_as<std::ranges::range_value_t<SegmentRange>, CircuitSegment>
        std::unique_ptr<CircuitQuadTreeNode>
        CreateTree(CircuitQuadTreeNode* parent, const Box2D& box, const SegmentRange& segments) {
            if (!IsUnitBox(box) && segments.begin() != segments.end()) {
                auto node = std::make_unique<CircuitQuadTreeNode>(CircuitQuadTreeInternalNode{}, box, parent);

                CircuitQuadTreeInternalNode state;
                for (auto direction : GetAllOrdinalDirections()) {
                    const auto childBox = CreateQuadrant(box, direction);
                    state.GetChild(direction) = CreateTree(
                        node.get(), childBox, segments | std::views::filter([&childBox](const auto& segment) {
                                                  return IntersectsBox(segment, childBox);
                                              }) | std::ranges::to<std::vector>()
                    );
                }
                node->State = std::move(state);
                return node;
            }

            if (IsUnitBox(box)) {
                const auto interiorSegments
                    = std::views::filter(segments, [&box](const auto& s) { return IntersectsUnitBoxInterior(s, box); })
                      | std::ranges::to<std::vector>();
                COMPG_ASSERT(interiorSegments.size() <= 1, "Expected at most one interior segment");
                const auto has45DegreeSegment = std::ranges::any_of(interiorSegments, IsAngle45);
                const auto has135DegreeSegment = std::ranges::any_of(interiorSegments, IsAngle135);
                COMPG_ASSERT(!(has45DegreeSegment && has135DegreeSegment), "Received invalid circuit segments");

                return std::make_unique<CircuitQuadTreeNode>(
                    CircuitQuadTreeLeafNode{
                        (has45DegreeSegment || has135DegreeSegment) ? std::optional{has45DegreeSegment} : std::nullopt
                    },
                    box, parent
                );
            }
            return std::make_unique<CircuitQuadTreeNode>(CircuitQuadTreeLeafNode{std::nullopt}, box, parent);
        }

        Box2D CreateBox(std::size_t power) {
            COMPG_ASSERT(std::numeric_limits<UnsignedInteger>::digits - 1 >= power, "The box is too large.");
            const UnsignedInteger u = 1 << power;
            return {{0, 0}, {u, u}};
        }

        Vertex2D Round(const Vertex2D& vertex) {
            return {static_cast<UnsignedInteger>(vertex[0]), static_cast<UnsignedInteger>(vertex[1])};
        }

        auto GetRoundedCorners(const Box2D& box) {
            auto corners = GetCorners(box);
            std::ranges::transform(corners, corners.begin(), Round);
            return corners;
        }

        auto GetEdgeCenters(const std::array<Vertex2D, 4>& corners) {
            std::array<Vertex2D, 4> edgeCenters;
            for (std::size_t i = 0; i < corners.size(); ++i) {
                std::size_t j = (i + 1) % corners.size();

                const auto middle = Round(math::LinearInterpolate(corners.at(i), corners.at(j), 0.5));
                edgeCenters.at(i) = middle;
            }

            return edgeCenters;
        }

        struct CreateSubdivision {
            void Call(const std::unique_ptr<CircuitQuadTreeNode>& node) const {
                operator()(node);
            }

            void operator()(const std::unique_ptr<CircuitQuadTreeNode>& node) const {
                std::visit(
                    Overloads{
                        [this, &node](const CircuitQuadTreeInternalNode& interiorNode) {
                            const auto centerIndex = Result.InsertVertex(Round(node->Box.GetCenter()));
                            const auto roundedCorners = GetRoundedCorners(node->Box);
                            const auto edgeCenters = GetEdgeCenters(roundedCorners);

                            for (std::size_t i = 0; i < roundedCorners.size(); ++i) {
                                std::size_t j = (i + 1) % roundedCorners.size();

                                const auto v0 = Result.GetVertexIndex(roundedCorners.at(i));
                                const auto v1 = Result.GetVertexIndex(roundedCorners.at(j));
                                const auto& middle = edgeCenters.at(i);

                                if (!Result.HasVertex(middle)) {
                                    Result.Split(v0, v1, middle);
                                }
                                const auto middleIndex = Result.GetVertexIndex(middle);
                                Result.InsertEdge(centerIndex, middleIndex);
                            }

                            for (const auto& child : interiorNode.Children) {
                                Call(child);
                            }
                        },
                        [](const CircuitQuadTreeLeafNode&) {}
                    },
                    node->State
                );
            }

            DoublyConnectedEdgeList& Result;
        };

        struct TriangulateSubdivision {
            void Call(const std::unique_ptr<CircuitQuadTreeNode>& node) const {
                operator()(node);
            }

            void operator()(const std::unique_ptr<CircuitQuadTreeNode>& node) const {
                std::visit(
                    Overloads{
                        [this](const CircuitQuadTreeInternalNode& interiorNode) {
                            for (const auto& child : interiorNode.Children) {
                                Call(child);
                            }
                        },
                        [this, &node](const CircuitQuadTreeLeafNode& leafNode) {
                            const auto roundedCorners = GetRoundedCorners(node->Box);

                            if (leafNode.IsInteriorSegmentAngle45.has_value()) {
                                HandleIntersectedLeafNode(leafNode, roundedCorners);
                            } else {
                                const auto edgeCenters = GetEdgeCenters(roundedCorners);
                                const bool onlyCorners
                                    = IsUnitBox(node->Box) || std::ranges::none_of(edgeCenters, [this](const auto& v) {
                                          return Result.HasVertex(v);
                                      });
                                if (onlyCorners) {
                                    const auto v0 = Result.GetVertexIndex(roundedCorners.at(0));
                                    const auto v1 = Result.GetVertexIndex(roundedCorners.at(2));
                                    Result.InsertEdge(v0, v1);
                                } else {
                                    HandleSteinerPoint(node, roundedCorners, edgeCenters);
                                }
                            }
                        }
                    },
                    node->State
                );
            }

        private:
            void HandleIntersectedLeafNode(
                const CircuitQuadTreeLeafNode& leafNode, const std::array<Vertex2D, 4>& roundedCorners
            ) const {
                const auto [i, j]
                    = leafNode.IsInteriorSegmentAngle45.value() ? std::tuple{0UZ, 2UZ} : std::tuple{1UZ, 3UZ};
                const auto v0 = Result.GetVertexIndex(roundedCorners.at(i));
                const auto v1 = Result.GetVertexIndex(roundedCorners.at(j));
                Result.InsertEdge(v0, v1);
            }

            void HandleSteinerPoint(
                const std::unique_ptr<CircuitQuadTreeNode>& node, const std::array<Vertex2D, 4>& roundedCorners,
                const std::array<Vertex2D, 4>& edgeCenters
            ) const {
                const auto centerIndex = Result.InsertVertex(Round(node->Box.GetCenter()));
                for (const auto& corner : roundedCorners) {
                    const auto cornerIndex = Result.GetVertexIndex(corner);
                    Result.InsertEdge(centerIndex, cornerIndex);
                }
                for (const auto& middle :
                     edgeCenters | std::views::filter([this](const auto& m) { return Result.HasVertex(m); })) {
                    const auto middleIndex = Result.GetVertexIndex(middle);
                    Result.InsertEdge(centerIndex, middleIndex);
                }
            }

        public:
            DoublyConnectedEdgeList& Result;
        };
    } // namespace details

    CircuitQuadTree::CircuitQuadTree(const std::vector<CircuitSegment>& segments, std::size_t power)
        : CircuitQuadTree{segments, details::CreateBox(power)} {}

    CircuitQuadTree::CircuitQuadTree(const std::vector<CircuitSegment>& segments, const Box2D& box) {
        std::ranges::for_each(segments, [&box](const auto& s) {
            // This is pretty retarded
            const Vertex2D v0{s[0][0], s[0][1]}; // = s[0].cast<Float>();
            const Vertex2D v1{s[1][0], s[1][1]};
            COMPG_ASSERT(box.Contains(v0) && box.Contains(v1), "Expected the box to contain the segments");
        });
        Root = details::CreateTree(nullptr, box, segments);
    }

    CircuitQuadTree::leaf_node_type
    CircuitQuadTree::CreateLeafNode(const node_type& presplitNode, const Box2D&, OrdinalDirection) {
        const leaf_node_type& presplitLeafNode = std::get<1>(presplitNode.State);
        COMPG_ASSERT(
            !presplitLeafNode.IsInteriorSegmentAngle45.has_value(),
            "Either a unit box is being split or a non unit box is intersected by a segment."
        );
        return leaf_node_type{};
    }

    DoublyConnectedEdgeList CircuitQuadTree::CreateMesh() {
        // TODO: the faces are invalid
        MakeBalanced();
        auto result = ConvertTo<DoublyConnectedEdgeList>(Polygon{GetCorners(Root->Box)});
        details::CreateSubdivision create{result};
        create.Call(Root);
        details::TriangulateSubdivision triangulate{result};
        triangulate.Call(Root);
        return result;
    }
} // namespace compg
