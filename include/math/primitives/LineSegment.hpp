#pragma once

#include "common/Common.hpp"
#include "common/Error.hpp"
#include "common/Vertex.hpp"
#include "math/Math.hpp"

namespace compg {

    template <typename V>
    class LineSegment {
    public:
        using array_type = std::array<V, 2>;

        LineSegment(const V& v1, const V& v2)
            : Vertices{v1, v2} {}

        explicit LineSegment(const array_type& vertices)
            : Vertices{vertices} {}

        bool IsValid(Float epsilon = EPSILON) const {
            return !AreEqual(Vertices[0], Vertices[1], epsilon);
        }

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

        bool operator==(const LineSegment& rhs) const {
            return Vertices.at(0) == rhs.Vertices.at(0) && Vertices.at(1) == rhs.Vertices.at(1);
        }

    private:
        array_type Vertices;
    };

    using LineSegment2D = LineSegment<Vertex2D>;
    using LineSegment3D = LineSegment<Vertex3D>;

    template <typename V>
    bool AreEqual(const LineSegment<V>& s1, const LineSegment<V>& s2, Float epsilon = EPSILON) {
        return AreEqual(s1[0], s2[0], epsilon) && AreEqual(s1[1], s2[1], epsilon);
    }
} // namespace compg

namespace std {
    template <size_t K>
    struct hash<compg::LineSegment<compg::Vertex<K>>> {
        size_t operator()(const compg::LineSegment<compg::Vertex<K>>& segment) const noexcept {
            size_t seed{0};
            compg::hash_combine(seed, segment[0]);
            compg::hash_combine(seed, segment[1]);
            return seed;
        }
    };
} // namespace std
