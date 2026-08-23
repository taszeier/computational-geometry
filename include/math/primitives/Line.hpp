#pragma once

#include "common/Common.hpp"
#include "common/Error.hpp"
#include "common/Vertex.hpp"
#include "math/Math.hpp"

namespace compg {

    template <typename V>
    class Line {
    public:
        using array_type = std::array<V, 2>;

        Line(const V& v1, const V& v2)
            : Vertices{v1, v2} {}

        explicit Line(const array_type& vertices)
            : Vertices{vertices} {}

        V LinearInterpolate(Float t) const {
            return math::LinearInterpolate(Vertices[0], Vertices[1], t);
        }

        const V& operator[](std::size_t index) const {
            COMPG_ASSERT(
                index < Vertices.size(), std::format(
                                             "parameter `index` is out of range: received value {} but "
                                             "expected value between 0 and {} (exclusive)",
                                             index, Vertices.size()
                                         )
            );
            return Vertices[index];
        }

    private:
        array_type Vertices;
    };

    using Line2D = Line<Vertex2D>;
    using Line3D = Line<Vertex3D>;

    inline Line2D MakeLine2DFromX(Float x) {
        return Line2D{
            Vertex2D{x, 0},
            Vertex2D{x, 1},
        };
    }

    inline Line2D MakeLine2DFromY(Float y) {
        return Line2D{
            Vertex2D{0, y},
            Vertex2D{1, y},
        };
    }
} // namespace compg
