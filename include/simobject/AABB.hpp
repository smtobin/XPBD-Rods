#pragma once

#include "common/common.hpp"

namespace SimObject
{

/** Axis-Aligned Bounding Box (AABB) */
struct AABB
{
    Vec3r min;
    Vec3r max;

    static AABB empty()
    {
        return { Vec3r::Constant(std::numeric_limits<Real>::max()), Vec3r::Constant(std::numeric_limits<Real>::lowest()) };
    }

    void expand(const Vec3r& p)
    {
        min = min.cwiseMin(p);
        max = max.cwiseMax(p);
    }

    void expand(const AABB& b)
    {
        min = min.cwiseMin(b.min);
        max = max.cwiseMax(b.max);
    }

    void pad(Real margin)
    {
        min.array() -= margin;
        max.array() += margin;
    }

    Vec3r center()  const { return (min + max) * Real(0.5); }
    Vec3r extent()  const { return (max - min) * Real(0.5); }

    bool overlaps(const AABB& o) const
    {
        return (min.array() <= o.max.array()).all() &&
               (max.array() >= o.min.array()).all();
    }
};


inline std::ostream& operator<<(std::ostream& os, const SimObject::AABB& aabb)
{
    os << aabb.min.transpose() << " to " << aabb.max.transpose();
    return os;
}

} // namespace SimObject