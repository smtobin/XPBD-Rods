#pragma once
#include "common/common.hpp"

#include <array>

template <typename T, std::size_t N1, std::size_t N2,
          std::size_t... I1, std::size_t... I2>
constexpr std::array<T, N1 + N2> concat_arrays_impl(
    const std::array<T, N1>& a,
    const std::array<T, N2>& b,
    std::index_sequence<I1...>,
    std::index_sequence<I2...>)
{
    return { a[I1]..., b[I2]... };
}

template <typename T, std::size_t N1, std::size_t N2>
constexpr std::array<T, N1 + N2> concat_arrays(
    const std::array<T, N1>& a,
    const std::array<T, N2>& b)
{
    return concat_arrays_impl(
        a, b,
        std::make_index_sequence<N1>{},
        std::make_index_sequence<N2>{});
}