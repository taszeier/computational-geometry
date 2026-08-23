#pragma once
#include "math/primitives/LineSegment.hpp"

namespace compg {
    using CircuitVertex = Eigen::Matrix<unsigned int, 2, 1>;

    class CircuitSegment {
    public:
        using segment_type = LineSegment<CircuitVertex>;

        explicit CircuitSegment(const segment_type& segment)
            : Segment(segment) {
            COMPG_ASSERT(IsAngleValid(), "Expected the angle to be either 0, 45, 90 or 135 degrees.");
        }

        const auto& operator[](std::size_t i) const {
            return Segment[i];
        }

    private:
        [[nodiscard]] bool IsAngleValid() const;

    private:
        segment_type Segment;
    };

    bool IsAngle0(const CircuitSegment& segment);
    bool IsAngle45(const CircuitSegment& segment);
    bool IsAngle90(const CircuitSegment& segment);
    bool IsAngle135(const CircuitSegment& segment);
} // namespace compg
