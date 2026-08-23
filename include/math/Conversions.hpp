#pragma once

#include "data_structures/DoublyConnectedEdgeList.hpp"
#include "math/primitives/Box.hpp"
#include "math/primitives/HalfPlane.hpp"
#include "math/primitives/Line.hpp"
#include "math/primitives/Polygon.hpp"
#include "math/primitives/Triangle.hpp"
#include "math/primitives/UnboundedBox.hpp"

namespace compg {
    template <typename To, typename From, typename... Extra>
    struct Converter;

    template <typename To, typename From, typename... Extra>
    To ConvertTo(const From& from, Extra&&... extra) {
        return Converter<To, From, std::decay_t<Extra>...>::Convert(from, std::forward<Extra>(extra)...);
    }

    template <template <std::size_t K> typename To, typename From, typename... Extra>
    To<From::K> ConvertTo(const From& from, Extra&&... extra) {
        return ConvertTo<To<From::K>>(from, std::forward<Extra>(extra)...);
    }

    template <std::size_t K>
    struct Converter<UnboundedBox<K>, Box<K>> {
        static constexpr auto Convert(const Box<K>& box) {
            UnboundedBox<K> unboundedBox;
            std::ranges::for_each(Indices<K>, [&unboundedBox, &box](auto i) {
                unboundedBox.Intersect(AxesAlignedHyperplane<K>{i, box.GetLowerCorner()(i)}, Side::Positive);
                unboundedBox.Intersect(AxesAlignedHyperplane<K>{i, box.GetUpperCorner()(i)}, Side::Negative);
            });

            return unboundedBox;
        }
    };

    template <std::size_t K>
    struct Converter<Hyperplane<K>, AxesAlignedHyperplane<K>, Side> {
        static constexpr auto Convert(const AxesAlignedHyperplane<K>& plane, Side side) {
            const Vertex<K> canonical = CreateCanonicalVector<K>(plane.GetAxisIndex());
            const Vertex<K> origin = canonical * plane.GetIntersection();
            const Vertex<K> normal = canonical * (side == Side::Positive ? 1 : -1);
            return Hyperplane<K>{origin, normal};
        }
    };

    template <std::size_t K>
    struct Converter<Hyperplane<K>, AxesAlignedHyperplane<K>> {
        static constexpr auto Convert(const AxesAlignedHyperplane<K>& plane) {
            return Converter<Hyperplane<K>, AxesAlignedHyperplane<K>, Side>::Convert(plane, Side::Positive);
        }
    };

    template <>
    struct Converter<Line2D, Hyperplane<2UZ>> {
        static auto Convert(const Hyperplane<2UZ>& hyperplane) {
            const Vertex2D n{
                -hyperplane.GetNormal()[1],
                hyperplane.GetNormal()[0],
            };
            const Vertex2D& v0 = hyperplane.GetOrigin();
            const Vertex2D v1 = v0 + n;
            return Line2D{v0, v1};
        }
    };

    template <>
    struct Converter<Hyperplane<2UZ>, Line2D> {
        static auto Convert(const Line2D& segment) {
            const Vertex2D d = segment[1] - segment[0];
            const Vertex2D n{d[1], -d[0]};
            return Hyperplane2D{segment[0], n};
        }
    };

    template <>
    struct Converter<Line2D, AxesAlignedHyperplane<2UZ>> {
        static auto Convert(const AxesAlignedHyperplane<2UZ>& plane) {
            const Vertex2D v0 = CreateCanonicalVector<2UZ>(plane.GetAxisIndex()) * plane.GetIntersection();
            const Vertex2D v1 = v0 + CreateCanonicalVector<2UZ>((plane.GetAxisIndex() + 1) % 2);
            return Line2D{v0, v1};
        }
    };

    template <>
    struct Converter<DoublyConnectedEdgeList, Polygon> {
        static auto Convert(const Polygon& polygon) {
            // TODO: set faces
            DoublyConnectedEdgeList edgeList;
            const auto firstIndex = edgeList.InsertVertex(polygon.GetVertex(0));
            auto previousIndex = firstIndex;
            for (std::size_t i = 1; i < polygon.NumVertices(); ++i) {
                const auto currentIndex = edgeList.InsertVertex(polygon.GetVertex(i));
                edgeList.InsertEdge(previousIndex, currentIndex);
                previousIndex = currentIndex;
            }

            edgeList.InsertEdge(previousIndex, firstIndex);
            return edgeList;
        }
    };

    template <>
    struct Converter<Hyperplane3D, Triangle3D> {
        static auto Convert(const Triangle3D& triangle) {
            const Vertex3D normal = math::Cross(triangle[0], triangle[1], triangle[2]);
            return Hyperplane3D{triangle[0], normal};
        }
    };
} // namespace compg
