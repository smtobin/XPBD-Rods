#pragma once

#include "common/common.hpp"

/** Helper class with static functions for N-point Gaussian quadrature on [0,1] interval. */
template<int N>
class GaussQuadratureHelper
{
public:

static std::array<Real, N> points()
{
    if constexpr (N == 1)
    {
        return {0.5};
    }
    else if constexpr (N == 2)
    {
        return {0.5 - 0.5/std::sqrt(3.0), 0.5 + 0.5/std::sqrt(3.0)};
    }
    else if constexpr (N == 3)
    {
        return {0.5 - 0.5*std::sqrt(3.0/5.0), 0.5, 0.5 + 0.5*std::sqrt(3.0/5.0)};
    }
    else if constexpr (N == 4)
    {
        return {0.069431844202974, 0.330009478207572, 0.669990521792428, 0.930568155797026};
    }
    else
    {
        return std::array<Real, N>{};
    }
}

static std::array<Real, N> weights()
{
    if constexpr (N == 1)
    {
        return {1.0};
    }
    else if constexpr (N == 2)
    {
        return {0.5, 0.5};
    }
    else if constexpr (N == 3)
    {
        return {5.0/18.0, 8.0/18.0, 5.0/18.0};
    }
    else if constexpr (N == 4)
    {
        return {0.173927422568727, 0.326072577431273, 0.326072577431273, 0.173927422568727};
    }
    else
    {
        return std::array<Real, N>{};
    }
}

};