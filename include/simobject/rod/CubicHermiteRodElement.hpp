#pragma once

#include "common/common.hpp"
#include "common/math.hpp"
#include "simobject/OrientedParticle.hpp"
#include "simobject/Particle.hpp"
#include "simobject/rod/RodElementBase.hpp"

#include <array>

namespace SimObject
{

class CubicHermiteRodElement : public RodElement_Base
{
public:
    static constexpr int NumNodes = 2;  // number of nodes in the element
    static constexpr int NumNodeDerivatives = 2; // number of nodal DOF derivatives incorporated in rod element
    static constexpr int NumGP = 3;     // number of Gauss points per element
    static constexpr int Order = 3;     // the order of the polynomial bases

    using NodeArrayType = std::array<OrientedParticle*, NumNodes>;
    using DerivativeArrayType = std::array<Particle*, NumNodes>;
    using StrainGradientMatType = Eigen::Matrix<Real, 6, 6*NumNodes + 6*NumNodeDerivatives>;
    using ContactPointGradientMatType = Eigen::Matrix<Real, 3, 6*NumNodes>;

    CubicHermiteRodElement(const NodeArrayType& nodes_list, 
        const DerivativeArrayType& dp_ds, const DerivativeArrayType& dR_ds,
        Real rest_length);

    virtual int order() const override { return Order; }

    static std::array<Real, 4> lumpedMasses();

    virtual OrientedParticle* node(int index) const override { return _nodes[index]; }
    virtual OrientedParticle* firstNode() const override { return _nodes.front(); }
    virtual OrientedParticle* lastNode() const override { return _nodes.back(); }
    const NodeArrayType& nodes() const { return _nodes; }
    const std::array<Particle*, 2*NumNodeDerivatives> nodeDerivatives() const { return {_dp_ds[0], _dR_ds[0], _dp_ds[1], _dR_ds[1]}; }

    const DerivativeArrayType& dpDOF() const { return _dp_ds; }
    const DerivativeArrayType& dRDOF() const { return _dR_ds; }

    virtual Real restLength() const override { return _rest_length; }

    virtual Vec3r position(Real s_hat) const override;
    virtual Mat3r orientation(Real s_hat) const override;

    Vec3r previousPosition(Real s_hat) const;
    Mat3r previousOrientation(Real s_hat) const;

    /** First arc length derivative w.r.t. reference coordinate s_hat of position */
    virtual Vec3r dposition_dshat(Real s_hat) const override;

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
    /** Array of pointers to nodes (oriented particles)
     * The source of p, R for the element endpoints
     */
    NodeArrayType _nodes;

    /** Array of points to h*p' for the element endpoints
     * This is stored in the rod class itself (since it is shared by multiple elements)
     * 
     * Note that the p' is assumed to be non-dimensionalized, i.e. multiplied by h
     * so that it has the same units as p. This helps scaling/conditioning.
     */
    DerivativeArrayType _dp_ds;

    /** Array of pointers to R' for the element endpoints
     * This is stored in the rod class itself (since it is ahred by multiple elements)
     * 
     * Note that the R' is assumed to be non-dimensionalized, i.e. multiplied by h
     * so that it has the same units as R. This helps scaling/conditioning.
     */
    DerivativeArrayType _dR_ds;


    /** Basis functions 
     * Because p' and R' have been scaled by h, all basis functions are non-dimensional scalar functions.
    */
    std::array<Real(*)(Real), 4> _bases;
    /** Basis function derivatives (with respect to s_hat) */
    std::array<Real(*)(Real), 4> _bases_derivatives;
};

} // namespace SimObject