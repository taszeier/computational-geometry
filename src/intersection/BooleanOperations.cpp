#include "intersection/BooleanOperations.hpp"

#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include "intersection/OverlayCalculator.hpp"
#include "math/primitives/Triangle.hpp"
#include "point_location/PointLocator.hpp"

namespace compg {
    namespace details {
        using face_index = DoublyConnectedEdgeList::face_index;

        Vertex2D FindVertexOnFace(const DoublyConnectedEdgeList& edgeList, face_index faceIndex) {
            const auto outerComponent = edgeList.GetFace(faceIndex).OuterComponent;
            if (outerComponent) {
                const auto leftMostEdge = FindLeftMostEdge(edgeList, outerComponent.value());

                const auto i0 = edgeList.GetOriginIndex(edgeList.GetPreviousIndex(leftMostEdge));
                const auto v0 = edgeList.GetVertex(i0).Vertex;
                const auto i1 = edgeList.GetOriginIndex(leftMostEdge);
                const auto v1 = edgeList.GetVertex(i1).Vertex;
                const auto i2 = edgeList.GetDestinationIndex(leftMostEdge);
                const auto v2 = edgeList.GetVertex(i2).Vertex;
                const Triangle2D triangle{v0, v1, v2};

                std::optional<Vertex2D> triangleLeftMost;
                auto visitEdge = [&edgeList, &triangle, &triangleLeftMost, i0, i1, i2](auto e) {
                    const auto originIndex = edgeList.GetOriginIndex(e);
                    const auto origin = edgeList.GetVertex(originIndex).Vertex;

                    if (originIndex == i0 || originIndex == i1 || originIndex == i2) {
                        return;
                    }

                    if (!Contains(triangle, origin)) {
                        return;
                    }

                    if (!triangleLeftMost || LexicographicalLess<Vertex2D>::Compare(origin, triangleLeftMost.value())) {
                        triangleLeftMost = origin;
                    }
                };

                WalkBoundary(edgeList, outerComponent.value(), visitEdge);
                for (const auto innerComponent : edgeList.GetFace(faceIndex).InnerComponents) {
                    WalkBoundary(edgeList, innerComponent, visitEdge);
                }

                return triangleLeftMost ? (triangleLeftMost.value() + v1) * 0.5 : triangle.Centroid();
            }

            std::optional<Vertex2D> leftMost;
            for (const auto innerComponent : edgeList.GetFace(faceIndex).InnerComponents) {
                WalkBoundary(edgeList, innerComponent, [&edgeList, &leftMost](auto e) {
                    const auto originIndex = edgeList.GetOriginIndex(e);
                    const auto origin = edgeList.GetVertex(originIndex).Vertex;

                    if (!leftMost || LexicographicalLess<Vertex2D>::Compare(origin, leftMost.value())) {
                        leftMost = origin;
                    }
                });
            }

            return leftMost.transform([](const auto& v) { return Vertex2D{v[0] - 1, v[1]}; }).value_or(Vertex2D{0, 0});
        }

        std::unordered_map<face_index, std::tuple<face_index, face_index>> FindCorrespondingFaces(
            const DoublyConnectedEdgeList& overlay, const DoublyConnectedEdgeList& edgeList1,
            const DoublyConnectedEdgeList& edgeList2
        ) {

            PointLocator locator1{edgeList1};
            PointLocator locator2{edgeList2};
            std::unordered_map<face_index, std::tuple<face_index, face_index>> result;

            for (face_index faceIndex = 0; faceIndex < overlay.NumFaces(); ++faceIndex) {
                const auto vertex = FindVertexOnFace(overlay, faceIndex);

                const auto location1 = locator1.LocatePoint(vertex);
                COMPG_ASSERT(location1.State.index() == 0, "Expected the vertex to be on a face of the edge list");
                const auto faceIndex1 = std::get<0>(location1.State);
                const auto location2 = locator2.LocatePoint(vertex);
                COMPG_ASSERT(location2.State.index() == 0, "Expected the vertex to be on a face of the edge list");
                const auto faceIndex2 = std::get<0>(location2.State);
                result.insert(std::pair{faceIndex, std::tuple{faceIndex1, faceIndex2}});
            }

            return result;
        }

        void
        TransferFace(const DoublyConnectedEdgeList& edgeList, face_index faceIndex, DoublyConnectedEdgeList& output) {
            COMPG_ASSERT(edgeList.GetFace(faceIndex).OuterComponent.has_value(), "Expected a bounded face");
            const auto outerComponent = edgeList.GetFace(faceIndex).OuterComponent.value();

            WalkBoundary(edgeList, outerComponent, [&edgeList, &output](auto edgeIndex) {
                const auto originIndex = edgeList.GetOriginIndex(edgeIndex);
                const auto origin = edgeList.GetVertex(originIndex).Vertex;
                const auto destinationIndex = edgeList.GetDestinationIndex(edgeIndex);
                const auto destination = edgeList.GetVertex(destinationIndex).Vertex;

                const auto i0 = output.InsertVertex(origin);
                const auto i1 = output.InsertVertex(destination);
                output.InsertEdge(i0, i1);
            });
        }

        DoublyConnectedEdgeList TransferOverlayFaces(
            const DoublyConnectedEdgeList& edgeList1, const DoublyConnectedEdgeList& edgeList2, auto&& keepFace
        ) {
            const OverlayCalculator calculator;
            const auto overlay = calculator.FindOverlay(edgeList1, edgeList2);
            const auto faceMap = details::FindCorrespondingFaces(overlay, edgeList1, edgeList2);

            DoublyConnectedEdgeList result;
            for (const auto& [faceIndex, tuple] : faceMap) {
                const auto& [faceIndex1, faceIndex2] = tuple;
                if (keepFace(faceIndex1, faceIndex2)) {
                    details::TransferFace(overlay, faceIndex, result);
                }
            }

            UpdateFaces(result);
            return result;
        }

    } // namespace details

    DoublyConnectedEdgeList Union(const DoublyConnectedEdgeList& edgeList1, const DoublyConnectedEdgeList& edgeList2) {
        // TODO: this is the same as just overlaying them. Return the face indices that are bounded in both input edge
        // lists.
        COMPG_ASSERT(edgeList1.AreFacesValid(), "Expected the edge list to have valid faces.");
        COMPG_ASSERT(edgeList2.AreFacesValid(), "Expected the edge list to have valid faces.");
        auto predicate =
            [unboundedFace1 = edgeList1.FindUnboundedFaceIndex(), unboundedFace2 = edgeList2.FindUnboundedFaceIndex()](
                DoublyConnectedEdgeList::face_index faceIndex1, DoublyConnectedEdgeList::face_index faceIndex2
            ) { return faceIndex1 != unboundedFace1 || faceIndex2 != unboundedFace2; };
        return details::TransferOverlayFaces(edgeList1, edgeList2, predicate);
    }

    DoublyConnectedEdgeList
    Intersection(const DoublyConnectedEdgeList& edgeList1, const DoublyConnectedEdgeList& edgeList2) {
        COMPG_ASSERT(edgeList1.AreFacesValid(), "Expected the edge list to have valid faces.");
        COMPG_ASSERT(edgeList2.AreFacesValid(), "Expected the edge list to have valid faces.");
        auto predicate =
            [unboundedFace1 = edgeList1.FindUnboundedFaceIndex(), unboundedFace2 = edgeList2.FindUnboundedFaceIndex()](
                DoublyConnectedEdgeList::face_index faceIndex1, DoublyConnectedEdgeList::face_index faceIndex2
            ) { return faceIndex1 != unboundedFace1 && faceIndex2 != unboundedFace2; };
        return details::TransferOverlayFaces(edgeList1, edgeList2, predicate);
    }

    DoublyConnectedEdgeList
    Difference(const DoublyConnectedEdgeList& edgeList1, const DoublyConnectedEdgeList& edgeList2) {
        COMPG_ASSERT(edgeList1.AreFacesValid(), "Expected the edge list to have valid faces.");
        COMPG_ASSERT(edgeList2.AreFacesValid(), "Expected the edge list to have valid faces.");
        auto predicate =
            [unboundedFace1 = edgeList1.FindUnboundedFaceIndex(), unboundedFace2 = edgeList2.FindUnboundedFaceIndex()](
                DoublyConnectedEdgeList::face_index faceIndex1, DoublyConnectedEdgeList::face_index faceIndex2
            ) { return faceIndex1 != unboundedFace1 && faceIndex2 == unboundedFace2; };
        return details::TransferOverlayFaces(edgeList1, edgeList2, predicate);
    }
} // namespace compg
