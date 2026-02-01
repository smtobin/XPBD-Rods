#pragma once

#include "collision/sdf/SDF.hpp"

namespace Collision
{

/** Implements a signed distance function for a rigid box. */
class BoxSDF : public SDF
{
    public:
    BoxSDF(const SimObject::XPBDRigidBox* box);

    /** Evaluates F(x) for a box with arbitrary position and orientation and size
     * @param x - the point at which to evaluate the SDF
     * @returns the distance from x to the shape boundary ( F(x) )
     */
    virtual Real evaluate(const Vec3r& x) const override;

    /** Evaluates the gradient of F at x.
     * @param x - the point at which to evaluate the graient of the SDF
     * @returns the gradient of the SDF at x.
     */
    virtual Vec3r gradient(const Vec3r& x) const override;

    const SimObject::XPBDRigidBox* box() const { return _box; }

    protected:
    /** Pointer to box needed for box's current position, orientation and size */
    const SimObject::XPBDRigidBox* _box;

};

} // namespace Collision