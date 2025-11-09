#pragma once

#include "common/common.hpp"

namespace Constraint
{

class XPBDConstraintProjector_Base
{
public:
    XPBDConstraintProjector_Base(Real dt)
        : _dt(dt)
    {}

    virtual void initialize() = 0;

    virtual void project() = 0;

protected:
    Real _dt;
};

} // namespace Constraint