#pragma once

#include "common/common.hpp"

namespace SimObject
{

struct AABB
{
    Vec3r min;
    Vec3r max;

    AABB()
    {
        min = std::numeric_limits<Real>::max() * Vec3r::Ones();
        max = std::numeric_limits<Real>::lowest() * Vec3r::Ones();
    }

    AABB(const Vec3r& min_, const Vec3r& max_)
        : min(min_), max(max_)
    {}
};

} // namespace SimObject