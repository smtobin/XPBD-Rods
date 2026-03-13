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
    RodElement_Base(Real rest_length)
        : _rest_length(rest_length)
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

    /** Position evaluated at the specified reference coordinate. (i.e. p(s_hat)) */
    virtual Vec3r position(Real s_hat) const = 0;
    /** Rotation evaluated at the specified reference coordinate. (i.e. R(s_hat)) */
    virtual Mat3r orientation(Real s_hat) const = 0;

    /** First arc length derivative w.r.t. reference coordinate s_hat of position */
    virtual Vec3r dposition_dshat(Real s_hat) const = 0;

    /** Second arc length derivative w.r.t. reference coordiante s_hat of position. */
    virtual Vec3r d2position_dshat2(Real s_hat) const = 0;

    /** Evaluates the shape function at the specified referenece coordinate. */
    virtual Real Ni(int shape_func_index, Real s_hat) const = 0;

    /** Evaluates the derivative of the shape function (w.r.t. the reference coordinate) at the specified reference coordinate. */
    virtual Real dNi_dshat(int shape_func_index, Real s_hat) const = 0;

    /** The derivative of the affine map s_hat w.r.t. s - this is just 1/l */
    virtual Real dshat_ds() const { return 1 / _rest_length; }

protected:
    Real _rest_length;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <int Order>
class RodElement : public RodElement_Base
{
public:
    using NodeArrayType = std::array<OrientedParticle*, Order+1>;
    using StrainGradientMatType = Eigen::Matrix<Real, 6, 6*(Order+1)>;
    using ContactPointGradientMatType = Eigen::Matrix<Real, 3, 6*(Order+1)>;

    RodElement(const NodeArrayType& nodes_list, Real rest_length);

    virtual int order() const override { return Order; }

    static std::array<Real, Order+1> lumpedMasses();

    virtual OrientedParticle* node(int index) const override { return _nodes[index]; }
    virtual OrientedParticle* firstNode() const override { return _nodes.front(); }
    virtual OrientedParticle* lastNode() const override { return _nodes.back(); }
    const NodeArrayType& nodes() const { return _nodes; }

    virtual Real restLength() const override { return _rest_length; }

    virtual Vec3r position(Real s_hat) const override;
    virtual Mat3r orientation(Real s_hat) const override;

    Vec3r previousPosition(Real s_hat) const;
    Mat3r previousOrientation(Real s_hat) const;

    /** First arc length derivative w.r.t. reference coordinate s_hat of position */
    virtual Vec3r dposition_dshat(Real s_hat) const override;

    /** Second arc length derivative w.r.t. reference coordiante s_hat of position. */
    virtual Vec3r d2position_dshat2(Real s_hat) const override;

    virtual Real Ni(int shape_func_index, Real s_hat) const override;
    virtual Real dNi_dshat(int shape_func_index, Real s_hat) const override;

    Vec3r shearStrain(Real s_hat) const;
    Vec3r bendingStrain(Real s_hat) const;

    Vec6r strain(Real s_hat) const;
    StrainGradientMatType strainGradient(Real s_hat) const;

    Vec3r contactPoint(Real s_hat, const Vec3r& cp_local) const;
    ContactPointGradientMatType contactPointGradient(Real s_hat, const Vec3r& cp_local) const;
    Vec3r previousContactPoint(Real s_hat, const Vec3r& cp_local) const;
    Vec3r contactPointVelocity(Real s_hat, const Vec3r& cp_local) const;


private:
    NodeArrayType _nodes;
    
    std::array<Real(*)(Real), Order+1> _bases;
    std::array<Real(*)(Real), Order+1> _bases_derivatives;
    std::array<Real(*)(Real), Order+1> _bases_derivatives2;
};

 /** Finds closest points between two rod elements. Use Newton's method to solve the optimization problem that minimizes squared error. */
std::vector<std::pair<Real,Real>> closestPointsBetweenRodElements(const RodElement_Base* elem1, const RodElement_Base* elem2);

} // namespace SimObject