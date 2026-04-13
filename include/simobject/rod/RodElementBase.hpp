#pragma once

#include "common/common.hpp"
#include "common/math.hpp"
#include "simobject/OrientedParticle.hpp"

#include <array>

namespace SimObject
{


/** Polymorphic RodElement base type.
 * Provides interface for position and rotation (p(s) and R(s)) and their derivatives.
 */
class RodElement_Base
{
public:
    static constexpr int NumNodes = 0;
    static constexpr int NumNodeDerivatives = 0;
    static constexpr int NumGP = 0;

    RodElement_Base(Real rest_length, const Vec3r& curvature)
        : _rest_length(rest_length), _inv_rest_length(1.0/rest_length), _curvature(curvature)
    {}

    virtual ~RodElement_Base() = default;

    /** The "order" of the rod element.
     * 1 = linear
     * 2 = quadratic,
     * etc.
     */
    virtual int order() const = 0;

    /** Returns the node at the specified index in the element. */
    virtual OrientedParticle* node(int index) const = 0;

    virtual OrientedParticle* firstNode() const = 0;
    virtual OrientedParticle* lastNode() const = 0;

    /** Rest length of the element */
    virtual Real restLength() const { return _rest_length; }

    /** Rest curvature of the element */
    const Vec3r& curvature() const { return _curvature; }

    /** Position evaluated at the specified reference coordinate. (i.e. p(s_hat)) */
    virtual Vec3r position(Real s_hat) const = 0;
    /** Rotation evaluated at the specified reference coordinate. (i.e. R(s_hat)) */
    virtual Mat3r orientation(Real s_hat) const = 0;

    /** First arc length derivative w.r.t. reference coordinate s_hat of position */
    virtual Vec3r dposition_dshat(Real s_hat) const = 0;

    /** Evaluates the shape function at the specified referenece coordinate. */
    virtual Real Ni(int shape_func_index, Real s_hat) const = 0;

    /** Evaluates the derivative of the shape function (w.r.t. the reference coordinate) at the specified reference coordinate. */
    virtual Real dNi_dshat(int shape_func_index, Real s_hat) const = 0;

    /** The derivative of the affine map s_hat w.r.t. s - this is just 1/l */
    virtual Real dshat_ds() const { return _inv_rest_length; }

protected:
    Real _rest_length;
    Real _inv_rest_length;

    Vec3r _curvature;
};

} // namespace SimObject