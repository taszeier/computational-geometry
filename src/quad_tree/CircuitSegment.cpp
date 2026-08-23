#include "quad_tree/CircuitSegment.hpp"

namespace compg {
    bool CircuitSegment::IsAngleValid() const {
        return IsAngle0(*this) || IsAngle45(*this) || IsAngle90(*this) || IsAngle135(*this);
    }

    bool IsAngle90(const CircuitSegment& segment) {
        return segment[0][0] == segment[1][0] && segment[0][1] != segment[1][1];
    }

    bool IsAngle0(const CircuitSegment& segment) {
        return segment[0][1] == segment[1][1] && segment[0][0] != segment[1][0];
    }

    bool IsAngle45(const CircuitSegment& segment) {
        const auto absDx = std::max(segment[0][0], segment[1][0]) - std::min(segment[0][0], segment[1][0]);
        const auto absDy = std::max(segment[0][1], segment[1][1]) - std::min(segment[0][1], segment[1][1]);
        return absDx == absDy
               && ((segment[0][0] < segment[1][0] && segment[0][1] < segment[1][1])
                   || (segment[0][0] > segment[1][0] && segment[0][1] > segment[1][1]));
    }

    bool IsAngle135(const CircuitSegment& segment) {
        const auto absDx = std::max(segment[0][0], segment[1][0]) - std::min(segment[0][0], segment[1][0]);
        const auto absDy = std::max(segment[0][1], segment[1][1]) - std::min(segment[0][1], segment[1][1]);
        return absDx == absDy
               && ((segment[0][0] < segment[1][0] && segment[0][1] > segment[1][1])
                   || (segment[0][0] > segment[1][0] && segment[0][1] < segment[1][1]));
    }
} // namespace compg