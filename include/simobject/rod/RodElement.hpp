#pragma once

#include "common/common.hpp"
#include "common/math.hpp"
#include "simobject/OrientedParticle.hpp"

#include <array>

namespace SimObject
{

template <int Order>
class RodElement
{
public:
    using StrainGradientMatType = Eigen::Matrix<Real, 6, 6*(Order+1)>;

    RodElement(std::initializer_list<const SimObject::OrientedParticle*> nodes_list, Real rest_length);

    Vec3r position(Real s_hat) const;
    Mat3r orientation(Real s_hat) const;

    Real Ni(int shape_func_index, Real s_hat) const;
    Real dNi_dshat(int shape_func_index, Real s_hat) const;
    Real dshat_ds() const { return 1/_rest_length; }

    Vec3r shearStrain(Real s_hat) const;
    Vec3r bendingStrain(Real s_hat) const;

    Vec6r strain(Real s_hat) const;
    StrainGradientMatType strainGradient(Real s_hat) const;
    

private:
    std::array<const SimObject::OrientedParticle*, Order+1> _nodes;
    
    std::array<Real(*)(Real), Order+1> _bases;
    std::array<Real(*)(Real), Order+1> _bases_derivatives;

    Real _rest_length;

};

} // namespace SimObject