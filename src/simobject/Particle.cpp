#include "simobject/Particle.hpp"

namespace SimObject
{
void Particle::inertialUpdate(Real dt, const Vec3r& F_ext)
{
    if (fixed)
        return;

    // positional update
    Vec3r m_inv = 1.0/mass.array();
    position += dt*velocity + dt*dt*m_inv.asDiagonal()*F_ext;
}

void Particle::inertialUpdate(Real dt)
{
    inertialUpdate(dt, Vec3r::Zero());
}

void Particle::inertialUpdateAcceleration(Real dt, const Vec3r& a)
{
    if (fixed)
        return;

    // positional update
    position += dt*velocity + dt*dt*a;
}

void Particle::positionUpdate(const Vec3r& dpos)
{
    if (fixed)
        return;
        
    position += dpos;
}

void Particle::velocityUpdate(Real dt)
{
    velocity = (position - prev_position)/dt;
    
    prev_position = position;
}

} // namespace SimObject