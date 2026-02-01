#pragma once

#include "collision/sdf/SDF.hpp"

namespace Collision
{

/** Implements a signed distance function for a rigid sphere. */
class SphereSDF : public SDF
{
    public:
    SphereSDF(const SimObject::XPBDRigidSphere* sphere);

    /** Evaluates F(x) for a sphere.
     * @param x - the point at which to evaluate the SDF
     * @returns the distance from x to the shape boundary ( F(x) )
    */
    virtual Real evaluate(const Vec3r& x) const override;

    /** Evaluates the gradient of F at x.
     * @param x - the point at which to evaluate the graient of the SDF
     * @returns the gradient of the SDF at x.
     */
    virtual Vec3r gradient(const Vec3r& x) const override;

    const SimObject::XPBDRigidSphere* sphere() const { return _sphere; }

    protected:
    /** Pointer to sphere needed for sphere's current position and radius. */
    const SimObject::XPBDRigidSphere* _sphere;
};

} // namespace Collision