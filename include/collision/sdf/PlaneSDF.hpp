#pragma once

#include "collision/sdf/SDF.hpp"

namespace Collision
{

/** Implements a signed distance function for a plnae.
 * Note: this models a plane as an impenetrable half-space, i.e. an object cannot go below the plane.
 */
class PlaneSDF : public SDF
{
    public:
    PlaneSDF(const SimObject::XPBDPlane* plane);

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

    const SimObject::XPBDPlane* plane() const { return _plane; }

    protected:
    /** Pointer to plane needed for plane's position and normal. */
    const SimObject::XPBDPlane* _plane;
};

} // namespace Collision