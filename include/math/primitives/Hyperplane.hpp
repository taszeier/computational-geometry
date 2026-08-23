#pragma once

#include "common/Common.hpp"
#include "common/Error.hpp"
#include "common/Vertex.hpp"
#include "math/Math.hpp"

namespace compg {
    template <std::size_t K>
    class Hyperplane {
    public:
        Hyperplane(const Vertex<K>& origin, const Vertex<K>& vector, Float epsilon = EPSILON) {
            COMPG_ASSERT(!math::IsZero(vector, epsilon), "Parameter `vector` cannot be 0");
            Origin = origin;
            Normal = vector.normalized();
        }

        const auto& GetOrigin() const {
            return Origin;
        }

        const auto& GetNormal() const {
            return Normal;
        }

        void Flip() {
            Normal *= -1;
        }

        bool operator==(const Hyperplane& other) const {
            return Origin == other.Origin && Normal == other.Normal;
        }

    private:
        Vertex<K> Origin;
        Vertex<K> Normal;
    };

    using Hyperplane2D = Hyperplane<2UZ>;
    using Hyperplane3D = Hyperplane<3UZ>;

    inline Hyperplane3D
    FindHyperplane(const Vertex3D& v0, const Vertex3D& v1, const Vertex3D& v2, Float epsilon = EPSILON) {
        const auto direction = math::Cross(v0, v1, v2);
        return Hyperplane3D{v0, direction, epsilon};
    }

    template <std::size_t K_>
    class AxesAlignedHyperplane {
    public:
        constexpr static std::size_t K = K_;

        AxesAlignedHyperplane(std::size_t axisIndex, Float intersection)
            : AxisIndex(axisIndex)
            , Intersection(intersection) {
            COMPG_ASSERT(axisIndex < K, CreateOutOfRangeMessage("axisIndex", axisIndex, K));
        }

        constexpr auto GetAxisIndex() const {
            return AxisIndex;
        }

        constexpr auto GetIntersection() const {
            return Intersection;
        }

    private:
        std::size_t AxisIndex;
        Float Intersection;
    };
} // namespace compg
