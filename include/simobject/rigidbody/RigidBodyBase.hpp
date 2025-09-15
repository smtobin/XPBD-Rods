#ifndef __RIGID_BODY_HPP
#define __RIGID_BODY_HPP

#include "common/common.hpp"

namespace RigidBody
{

class RigidBody_Base
{

public:
    RigidBody_Base(const std::string& name);

    /** Performs necessary setup to prepare the rod for simulation. (sets up constraints, computes mass properties, etc.) */
    void setup();

    /** Steps the rigid body forward in time by dt. */
    void update(Real dt, Real g_accel);

};


} // namespace RigidBody


#endif // __RIGID_BODY_HPP