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
    using NodeArrayType = std::array<SimObject::OrientedParticle*, Order+1>;
    using StrainGradientMatType = Eigen::Matrix<Real, 6, 6*(Order+1)>;

    RodElement(const NodeArrayType& nodes_list, Real rest_length);

    static std::array<Real, Order+1> lumpedMasses();

    const NodeArrayType& nodes() const { return _nodes; }
    Real restLength() const { return _rest_length; }

    Vec3r position(Real s_hat) const;
    Mat3r orientation(Real s_hat) const;

    /** First arc length derivative w.r.t. reference coordinate s_hat of position */
    Vec3r dposition_dshat(Real s_hat) const;

    /** Second arc length derivative w.r.t. reference coordiante s_hat of position. */
    Vec3r d2position_dshat2(Real s_hat) const;

    Real Ni(int shape_func_index, Real s_hat) const;
    Real dNi_dshat(int shape_func_index, Real s_hat) const;
    Real dshat_ds() const { return 1/_rest_length; }

    Vec3r shearStrain(Real s_hat) const;
    Vec3r bendingStrain(Real s_hat) const;

    Vec6r strain(Real s_hat) const;
    StrainGradientMatType strainGradient(Real s_hat) const;

private:
    NodeArrayType _nodes;
    
    std::array<Real(*)(Real), Order+1> _bases;
    std::array<Real(*)(Real), Order+1> _bases_derivatives;
    std::array<Real(*)(Real), Order+1> _bases_derivatives2;

    Real _rest_length;

};

 /** Finds closest points between two rod elements. Use Newton's method to solve the optimization problem that minimizes squared error. */
template <int Order1, int Order2>
void closestPointsBetweenRodElements(const RodElement<Order1>* elem1, const RodElement<Order2>* elem2);

} // namespace SimObject