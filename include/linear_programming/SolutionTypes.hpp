#pragma once

#include "common/Vertex.hpp"
#include "math/Math.hpp"

namespace compg {
    struct Infeasible {};

    struct Bounded {
        Vertex2D Solution;
    };
    struct Unbounded {
        Vertex2D Ray;
    };

    inline bool AreEqual(const Bounded& a, const Bounded& b, Float epsilon = EPSILON) {
        return AreEqual(a.Solution, b.Solution, epsilon);
    }

    inline bool AreEqual(const Unbounded& a, const Unbounded& b, Float epsilon = EPSILON) {
        return AreEqual(a.Ray.normalized(), b.Ray.normalized(), epsilon);
    }
} // namespace compg