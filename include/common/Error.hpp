#pragma once
#include <format>
#include <stacktrace>
#include <stdexcept>

namespace compg {
    class Exception : public std::runtime_error {
    public:
        Exception(std::string_view message, const std::stacktrace& stacktrace)
            : std::runtime_error(std::format("{}\n{}", message, std::to_string(stacktrace)))
            , Stacktrace(stacktrace) {}

        static std::string FormatErrorMessage(std::string_view message, const char* expression = nullptr) {
            if (expression == nullptr) {
                return std::format("Error: {}", message);
            }
            return std::format("Error in expression {}\n\t{}", expression, message);
        }

    private:
        std::stacktrace Stacktrace;
    };

    template <typename IndexType, typename BoundsType>
    std::string CreateOutOfRangeMessage(
        std::string_view parameterName, IndexType parameterValue, BoundsType lowerBound, BoundsType upperBound
    ) {
        return std::format(
            "parameter `{}` is out of range: received value {} but expected "
            "value between {} and {} (exclusive)",
            parameterName, parameterValue, lowerBound, upperBound
        );
    }

    template <typename IndexType, typename BoundsType>
    std::string
    CreateOutOfRangeMessage(std::string_view parameterName, IndexType parameterValue, BoundsType upperBound) {
        return CreateOutOfRangeMessage(parameterName, parameterValue, static_cast<BoundsType>(0), upperBound);
    }

} // namespace compg

#ifndef COMPG_ASSERT
#define COMPG_ASSERT(cond, msg)                                                                                        \
    {                                                                                                                  \
        if (!(cond)) {                                                                                                 \
            throw compg::Exception(compg::Exception::FormatErrorMessage(msg, #cond), std::stacktrace::current());      \
        }                                                                                                              \
    }
#endif

#ifndef COMPG_THROW
#define COMPG_THROW(msg)                                                                                               \
    {                                                                                                                  \
        throw compg::Exception(compg::Exception::FormatErrorMessage(msg), std::stacktrace::current());                 \
    }
#endif