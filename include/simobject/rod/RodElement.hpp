#pragma once

#include "common/common.hpp"
#include "common/math.hpp"
#include "simobject/OrientedParticle.hpp"
#include "simobject/Particle.hpp"
#include "simobject/rod/RodElementBase.hpp"

#include <array>

namespace SimObject
{

template <int Order_>
class RodElement : public RodElement_Base
{
public:
    static constexpr int NumNodes = std::max(2,Order_+1);  // number of nodes in the element
    static constexpr int NumGP = Order_;     // number of Gauss points per element
    static constexpr int Order = Order_;     // the order of the polynomial bases

    using NodeArrayType = std::array<OrientedParticle*, NumNodes>;
    using StrainGradientMatType = Eigen::Matrix<Real, 6, 6*(NumNodes)>;
    using ContactPointGradientMatType = Eigen::Matrix<Real, 3, 6*(NumNodes)>;

    RodElement(const NodeArrayType& nodes_list, Real rest_length);

    virtual int order() const override { return Order; }

    static std::array<Real, NumNodes> lumpedMasses();

    virtual OrientedParticle* node(int index) const override { return _nodes[index]; }
    virtual OrientedParticle* firstNode() const override { return _nodes.front(); }
    virtual OrientedParticle* lastNode() const override { return _nodes.back(); }
    const NodeArrayType& nodes() const { return _nodes; }
    std::array<Particle*, 0> nodeDerivatives() const { return {}; }

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
    NodeArrayType _nodes;
    
    std::array<Real(*)(Real), NumNodes> _bases;
    std::array<Real(*)(Real), NumNodes> _bases_derivatives;
};

} // namespace SimObject