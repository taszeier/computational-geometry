#include "triangulation/ArtGalleryProblem.hpp"
#include "data_structures/DoublyConnectedEdgeListAlgorithms.hpp"
#include "triangulation/PolygonTriangulator.hpp"

namespace compg {
    std::size_t ArtGalleryProblem::FindNumCameras(const DoublyConnectedEdgeList& polygon) const {
        const auto cameraMap = FindCameraMap(polygon);
        std::array<std::size_t, 3U> numCameras{};
        std::ranges::for_each(
            cameraMap, [&numCameras](auto i) { ++numCameras.at(i); }, [](auto pair) { return pair.second; }
        );
        return *std::ranges::min_element(numCameras);
    }

    std::unordered_map<DoublyConnectedEdgeList::vertex_index, std::size_t>
    ArtGalleryProblem::FindCameraMap(const DoublyConnectedEdgeList& polygon) const {
        auto polygonTriangulated{polygon};
        const PolygonTriangulator polygonTriangulator{};
        polygonTriangulator.Triangulate(polygonTriangulated);
        UpdateFaces(polygonTriangulated);

        std::unordered_map<DoublyConnectedEdgeList::vertex_index, std::size_t> cameraIndices;
        auto assignCamera = [&polygonTriangulated, &cameraIndices](auto faceIndex) {
            const auto face = polygonTriangulated.GetFace(faceIndex);
            if (face.OuterComponent.has_value()) {
                auto vertices = FindBoundaryVertices(polygonTriangulated, face.OuterComponent.value());
                COMPG_ASSERT(vertices.size() == 3, "Expected the face to be a triangle");

                auto v0It = cameraIndices.find(vertices.at(0));
                auto v1It = cameraIndices.find(vertices.at(1));
                auto v2It = cameraIndices.find(vertices.at(2));
                if (v0It == cameraIndices.end()) {
                    std::swap(v0It, v1It);
                    std::swap(vertices.at(0), vertices.at(1));
                }
                if (v1It == cameraIndices.end()) {
                    std::swap(v1It, v2It);
                    std::swap(vertices.at(1), vertices.at(2));
                }
                if (v2It == cameraIndices.end()) {
                    if (v0It != cameraIndices.end()) {
                        COMPG_ASSERT(v1It != cameraIndices.end(), "This should be impossible");
                        const auto v2CameraIndex = 3UZ - v0It->second - v1It->second;
                        cameraIndices[vertices.at(2)] = v2CameraIndex;
                    } else {
                        COMPG_ASSERT(v1It == cameraIndices.end(), "This should be impossible");
                        cameraIndices[vertices.at(0)] = 0UZ;
                        cameraIndices[vertices.at(1)] = 1UZ;
                        cameraIndices[vertices.at(2)] = 2UZ;
                    }
                }
            }
        };

        const DoublyConnectedEdgeList::face_index startIndex = polygonTriangulated.IsFaceBounded(0) ? 0 : 1;
        DepthFirstSearchBoundedFaces(polygonTriangulated, assignCamera, startIndex);
        return cameraIndices;
    }
} // namespace compg