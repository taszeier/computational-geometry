#include "CircuitSegmentValidator.hpp"

#include <algorithm>

using namespace compg;

namespace {
    bool AreSameClass(const CircuitSegment& s1, const CircuitSegment& s2) {
        if (IsAngle0(s1)) {
            return IsAngle0(s2) && s1[0][1] == s2[0][1];
        }
        if (IsAngle45(s1)) {
            return IsAngle45(s2) && (s1[0][0] + s2[0][1] == s2[0][0] + s1[0][1]);
        }
        if (IsAngle90(s1)) {
            return IsAngle90(s2) && s1[0][0] == s2[0][0];
        }
        if (IsAngle135(s1)) {
            return IsAngle135(s2) && (s1[0][0] + s1[0][1] == s2[0][0] + s2[0][1]);
        }
        COMPG_THROW("Received invalid segment");
    }

    bool ProjectionsOverlap(const CircuitSegment& s1, const CircuitSegment& s2) {
        long coordinateIndex = IsAngle90(s1) ? 1 : 0;
        const auto firstStart = s1[0][coordinateIndex];
        const auto firstEnd = s1[1][coordinateIndex];
        const auto secondStart = s2[0][coordinateIndex];
        const auto secondEnd = s2[1][coordinateIndex];
        const auto overlapStart = std::max(std::min(firstStart, firstEnd), std::min(secondStart, secondEnd));
        const auto overlapEnd = std::min(std::max(firstStart, firstEnd), std::max(secondStart, secondEnd));
        return overlapStart < overlapEnd;
    }

    bool Overlap(const CircuitSegment& s1, const CircuitSegment& s2) {
        return AreSameClass(s1, s2) && ProjectionsOverlap(s1, s2);
    }

    bool HasIntersection(const CircuitSegment& s45, const CircuitSegment& s135) {
        const bool bothSides45 = (s135[0][0] + s45[0][1] > s45[0][0] + s135[0][1] && s45[0][0] + s135[1][1] > s135[1][0] + s45[0][1])
            || (s135[0][0] + s45[0][1] < s45[0][0] + s135[0][1] && s45[0][0] + s135[1][1] < s135[1][0] + s45[0][1]);
        const bool bothSides135 = (s45[0][0] + s45[0][1] > s135[0][0] + s135[0][1] && s135[0][0] + s135[0][1] > s45[1][0] + s45[1][1])
            || (s45[0][0] + s45[0][1] < s135[0][0] + s135[0][1] && s135[0][0] + s135[0][1] < s45[1][0] + s45[1][1]);
        return bothSides45 && bothSides135;
    }

    bool HasNonIntegerIntersectionImpl(const CircuitSegment& s45, const CircuitSegment& s135) {
        return HasIntersection(s45, s135) && s45[0][1] % 2 != ((s135[0][0] + s135[0][1] % 2) + s45[0][0]) % 2;
    }

    bool HasNonIntegerIntersection(const CircuitSegment& s1, const CircuitSegment& s2) {
        if (IsAngle45(s1)) {
            return IsAngle135(s2) && HasNonIntegerIntersectionImpl(s1, s2);
        }
        return IsAngle45(s2) && IsAngle135(s1) && HasNonIntegerIntersectionImpl(s2, s1);
    }
} // namespace

std::string CircuitSegmentValidationResult::GetMessage() const {
    switch (Error) {
    case CircuitSegmentValidationError::None:
        return "Segment accepted";
    case CircuitSegmentValidationError::Degenerate:
        return "The endpoints must be different";
    case CircuitSegmentValidationError::OutsideBox:
        return "The segment must stay inside the circuit box";
    case CircuitSegmentValidationError::InvalidAngle:
        return "The segment angle must be 0, 45, 90, or 135 degrees";
    case CircuitSegmentValidationError::Overlap:
        return "Segments cannot overlap";
    case CircuitSegmentValidationError::NonIntegerIntersection:
        return "Segments cannot intersect at a non-integer point";
    }
    COMPG_THROW("Received invalid segment");
}

CircuitSegmentValidationResult ValidateCircuitSegment(
    const compg::CircuitSegment::segment_type& candidate,
    const std::vector<compg::CircuitSegment>& segments,
    std::size_t power
) {
    const auto upperBound = static_cast<CircuitVertex::Scalar>(1) << power;
    const auto& first = candidate[0];
    const auto& second = candidate[1];
    if (first == second) {
        return {CircuitSegmentValidationError::Degenerate};
    }
    if (first[0] > upperBound || first[1] > upperBound || second[0] > upperBound || second[1] > upperBound) {
        return {CircuitSegmentValidationError::OutsideBox};
    }

    auto maybeSegment = Try([](const auto& s) { return CircuitSegment{s}; }, candidate);
    if (!maybeSegment.has_value()) {
        return {CircuitSegmentValidationError::InvalidAngle};
    }
    const auto& candidateSegment = maybeSegment.value();
    for (const auto& existing : segments) {
        if (Overlap(candidateSegment, existing)) {
            return {CircuitSegmentValidationError::Overlap};
        }
        if (HasNonIntegerIntersection(candidateSegment, existing)) {
            return {CircuitSegmentValidationError::NonIntegerIntersection};
        }
    }
    return {};
}