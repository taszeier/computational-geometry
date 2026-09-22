#pragma once

#include "quad_tree/CircuitSegment.hpp"

#include <string>
#include <vector>

enum class CircuitSegmentValidationError {
    None,
    Degenerate,
    OutsideBox,
    InvalidAngle,
    Overlap,
    NonIntegerIntersection,
};

struct CircuitSegmentValidationResult {
    CircuitSegmentValidationError Error = CircuitSegmentValidationError::None;

    [[nodiscard]] bool IsValid() const {
        return Error == CircuitSegmentValidationError::None;
    }

    [[nodiscard]] std::string GetMessage() const;
};

[[nodiscard]] CircuitSegmentValidationResult ValidateCircuitSegment(
    const compg::CircuitSegment::segment_type& candidate, const std::vector<compg::CircuitSegment>& segments,
    std::size_t power
);