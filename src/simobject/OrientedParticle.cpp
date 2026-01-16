#include "simobject/OrientedParticle.hpp"

#include <Eigen/Geometry>

namespace SimObject
{
void OrientedParticle::inertialUpdate(Real dt, const Vec3r& F_ext, const Vec3r& T_ext)
{
    if (fixed)
        return;

    // positional update
    position += dt*lin_velocity + dt*dt/mass*F_ext;

    // rotational update
    Vec3r Ib_inv = 1.0/Ib.array();
    Vec3r so3_update = dt*ang_velocity + dt*dt*Ib_inv.asDiagonal()*(T_ext - ang_velocity.cross(Ib.asDiagonal()*ang_velocity));
    orientation = Math::Plus_SO3(orientation, so3_update);
}

void OrientedParticle::inertialUpdate(Real dt)
{
    inertialUpdate(dt, Vec3r::Zero(), Vec3r::Zero());
}

void OrientedParticle::positionUpdate(const Vec3r& dpos, const Vec3r& dor)
{
    if (fixed)
        return;
        
    position += dpos;
    orientation = Math::Plus_SO3(orientation, dor);
}

void OrientedParticle::positionUpdate(const Vec6r& dp)
{
    positionUpdate(dp.head<3>(), dp.tail<3>());
}

void OrientedParticle::velocityUpdate(Real dt, const Vec3r& prev_pos, const Mat3r& prev_or)
{
    lin_velocity = (position - prev_pos)/dt;
    ang_velocity = Math::Minus_SO3(orientation, prev_or)/dt;
}

} // namespace SimObject