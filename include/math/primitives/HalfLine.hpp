#pragma once
#include "common/Error.hpp"
#include "common/Vertex.hpp"
#include "math/Math.hpp"

namespace compg {
    template <std::size_t K>
    class HalfLine {
    public:
        HalfLine(const Vertex<K>& origin, const Vertex<K> direction)
            : Origin(origin)
            , Normal(direction) {

            COMPG_ASSERT(direction.squaredNorm() > EPSILON * EPSILON, "Parameter `direction` cannot be zero");
            Normal.normalize();
        }

        const auto& GetOrigin() const {
            return Origin;
        }

        const auto& GetNormal() const {
            return Normal;
        }

        Vertex<K> LinearInterpolate(Float t) const {
            return math::LinearInterpolate<Vertex<K>>(Origin, Origin + Normal, t);
        }

    private:
        Vertex<K> Origin;
        Vertex<K> Normal;
    };

    using HalfLine2D = HalfLine<2UZ>;
} // namespace compg
