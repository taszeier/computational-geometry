#pragma once

#include "common/Error.hpp"
#include <eigen3/Eigen/Core>

namespace compg {
    using Float = double;
    constexpr Float EPSILON = 1e-10;
    constexpr Float Infinity = std::numeric_limits<Float>::infinity();

    template <class... Ts>
    struct Overloads : Ts... {
        using Ts::operator()...;
    };

    template <std::integral T, T... i>
    constexpr auto ConvertToArray(std::integer_sequence<T, i...>) {
        return std::array{i...};
    }

    template <std::size_t K>
    inline constexpr auto Indices = ConvertToArray(std::make_index_sequence<K>{});

    struct Identity {
        template <typename T>
        T&& operator()(T&& t) const noexcept {
            return std::forward<T>(t);
        }
    };

    struct Less {
        template <typename T, typename U>
        bool operator()(T&& t, U&& u) const {
            return std::forward<T>(t) < std::forward<U>(u);
        }
    };

    struct Greater {
        template <typename T, typename U>
        bool operator()(T&& t, U&& u) const {
            return std::forward<T>(t) > std::forward<U>(u);
        }
    };

    template <typename FunctionType, typename TupleType, size_t... Is>
    void for_each_in_tuple_impl(FunctionType&& function, const TupleType& tuple, std::index_sequence<Is...>) {
        (function(Is, std::get<Is>(tuple)), ...);
    }

    template <typename FunctionType, typename... Args>
    void for_each_in_tuple(const std::tuple<Args...>& tuple, FunctionType&& function) {
        for_each_in_tuple_impl(
            std::forward<FunctionType>(function), tuple, std::make_index_sequence<sizeof...(Args)>()
        );
    }

    template <typename FunctionType, typename TupleType, size_t... Is>
    constexpr auto map_tuple_impl(FunctionType&& func, const TupleType& tuple, std::index_sequence<Is...>) {
        return std::make_tuple(func(Is, std::get<Is>(tuple))...);
    }

    template <typename FunctionType, typename... Args>
    constexpr auto map_tuple(const std::tuple<Args...>& tuple, FunctionType&& function) {
        return map_tuple_impl(std::forward<FunctionType>(function), tuple, std::make_index_sequence<sizeof...(Args)>{});
    }

    template <typename... Args, std::size_t... Is>
    bool tuple_equals_impl(const std::tuple<Args...>& t1, const std::tuple<Args...>& t2, std::index_sequence<Is...>) {
        return ((std::get<Is>(t1) == std::get<Is>(t2)) && ...);
    }

    template <typename... Args>
    bool operator==(const std::tuple<Args...>& tuple1, const std::tuple<Args...>& tuple2) {
        return tuple_equals_impl(tuple1, tuple2, std::index_sequence_for<Args...>{});
    }

    template <typename ContainerType>
    std::optional<typename ContainerType::value_type> At(const ContainerType& container, std::size_t index) {
        if (index < container.size()) {
            return std::optional{container.at(index)};
        }
        return std::nullopt;
    }

    /**
     * @brief Wrap a function call in a try-catch block.
     * @param function A callable object.
     * @param args The arguments to the object.
     * @return The result of the function if it succeeds or std::nullopt if it throws an exception.
     */
    template <typename... Args, typename FunctionType>
    auto Try(FunctionType&& function, Args&&... args)
        -> std::optional<decltype(function(std::forward<Args>(args)...))> {
        try {
            auto result = function(std::forward<Args>(args)...);
            return std::optional{std::move(result)};
        } catch (const Exception&) {
            return std::nullopt;
        }
    }

    template <typename ValueType, typename ComparatorType>
    const ValueType& Min(const ValueType& a, const ValueType& b, ComparatorType comparator = {}) {
        if (comparator(b, a)) {
            return b;
        }
        return a;
    }

    template <typename ValueType, typename ComparatorType>
    const ValueType& Max(const ValueType& a, const ValueType& b, ComparatorType comparator = {}) {
        if (comparator(a, b)) {
            return b;
        }
        return a;
    }
} // namespace compg