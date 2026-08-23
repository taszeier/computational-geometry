#pragma once
#include "common/Vertex.hpp"
#include "math/Math.hpp"

namespace compg {
    struct QuadraticPolynomial {
        Vertex3D Coefficients;

        [[nodiscard]] Float Evaluate(Float x) const {
            return Coefficients[0] + x * (Coefficients[1] + Coefficients[2] * x);
        }
    };

    inline QuadraticPolynomial operator+(const QuadraticPolynomial& a, const QuadraticPolynomial& b) {
        return {a.Coefficients + b.Coefficients};
    }

    inline QuadraticPolynomial operator-(const QuadraticPolynomial& a, const QuadraticPolynomial& b) {
        return {a.Coefficients - b.Coefficients};
    }

    /**
     * @brief The roots of a quadratic polynomial.
     * @details Contains two optional roots. If MaxRoot has a value then so does MinRoot and MinRoot is always less than
     * MaxRoot if they both have values.
     */
    struct QuadraticPolynomialRoots {
        std::optional<Float> MinRoot;
        std::optional<Float> MaxRoot;
    };

    inline QuadraticPolynomialRoots FindRealRoots(const QuadraticPolynomial& polynomial, Float epsilon = EPSILON) {
        Float a = polynomial.Coefficients[2];
        Float b = polynomial.Coefficients[1];
        Float c = polynomial.Coefficients[0];
        if (math::IsZero(a, epsilon)) {
            if (math::IsZero(b, epsilon)) {
                const auto minSolution = math::IsZero(c, epsilon) ? std::optional{0.0} : std::nullopt;
                return {.MinRoot = minSolution, .MaxRoot = std::nullopt};
            }
            return {.MinRoot = -c / b, .MaxRoot = std::nullopt};
        }
        const auto discriminant = b * b - 4 * a * c;
        if (math::IsZero(discriminant, epsilon)) {
            return {.MinRoot = -b * 0.5 / a, .MaxRoot = std::nullopt};
        }
        if (discriminant < 0) {
            return {.MinRoot = std::nullopt, .MaxRoot = std::nullopt};
        }
        const auto discriminantRoot = std::sqrt(discriminant);
        const auto [x1, x2] = std::tuple{(-b - discriminantRoot) * 0.5 / a, (-b + discriminantRoot) * 0.5 / a};
        return {.MinRoot = std::min(x1, x2), .MaxRoot = std::max(x1, x2)};
    }

} // namespace compg
