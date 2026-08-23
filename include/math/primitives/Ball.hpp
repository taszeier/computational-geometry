#pragma once
#include "common/Common.hpp"
#include "common/Vertex.hpp"
#include "math/Math.hpp"

namespace compg {
    template <std::size_t K>
    class Ball {
    public:
        Ball(const Vertex<K>& center, Float radius)
            : Center(center)
            , Radius(radius) {}

        [[nodiscard]] bool Contains(const Vertex<K>& vertex) const {
            return (Center - vertex).norm() <= Radius;
        }

        const Vertex<K>& GetCenter() const {
            return Center;
        }

        [[nodiscard]] Float GetRadius() const {
            return Radius;
        }

        bool operator==(const Ball& other) const {
            return Center == other.Center && Radius == other.Radius;
        }

    private:
        Vertex<K> Center;
        Float Radius;
    };

    template <std::size_t K>
    bool AreEqual(const Ball<K>& a, const Ball<K>& b, Float epsilon = EPSILON) {
        return AreEqual(a.GetCenter(), b.GetCenter(), epsilon) && math::IsZero(a.GetRadius() - b.GetRadius(), epsilon);
    }
} // namespace compg
