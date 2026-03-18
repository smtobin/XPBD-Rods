#include "simobject/rod/RodElement.hpp"

namespace SimObject
{

template<>
Vec3r RodElement<0>::position(Real s_hat) const
{
    std::cout << "RodElement<0>::position" << std::endl;
    // the current length of the element factors in the stretch
    // stretch strain ==> 3rd component of [ 1/l * 1/2 * (R1 + R2)^T (p2 - p1) ]
    // therefore the current length of the element is 3rd component of 1/2 * (R1 + R2)^T (p2 - p1)
    Vec3r stretch = 0.5*(_nodes[0]->orientation.transpose() + _nodes[1]->orientation.transpose()) * (_nodes[1]->position - _nodes[0]->position);
    Real current_length = stretch[2];

    // if s_hat < 0.5, position is on "rigid body" associated with first node
    // otherwise position is on "rigid body" associated with second node
    if (s_hat < 0.5)
    {
        return _nodes[0]->position + s_hat * current_length * _nodes[0]->orientation.col(2);
    }
    else
    {
        return _nodes[1]->position - (1-s_hat) * current_length * _nodes[1]->orientation.col(2);
    }
}

template<>
Vec3r RodElement<0>::previousPosition(Real s_hat) const
{
    // the current length of the element factors in the stretch
    // stretch strain ==> 3rd component of [ 1/l * 1/2 * (R1 + R2)^T (p2 - p1) ]
    // therefore the current length of the element is 3rd component of 1/2 * (R1 + R2)^T (p2 - p1)
    Vec3r prev_stretch = 0.5*(_nodes[0]->prev_orientation.transpose() + _nodes[1]->prev_orientation.transpose()) * (_nodes[1]->prev_position - _nodes[0]->prev_position);
    Real prev_length = prev_stretch[2];

    // if s_hat < 0.5, position is on "rigid body" associated with first node
    // otherwise position is on "rigid body" associated with second node
    if (s_hat < 0.5)
    {
        return _nodes[0]->prev_position + s_hat * prev_length * _nodes[0]->prev_orientation.col(2);
    }
    else
    {
        return _nodes[1]->prev_position - (1-s_hat) * prev_length * _nodes[1]->prev_orientation.col(2);
    }
}

template<>
Vec3r RodElement<0>::dposition_dshat(Real /* s_hat */) const
{
    // this doesn't make sense, throw an error
    throw std::runtime_error("dposition_dshat not defined for RodElement<0>");

    return Vec3r::Zero();
}

template<>
Vec3r RodElement<0>::d2position_dshat2(Real /* s_hat */) const
{
    // this doesn't make sense, throw an error
    throw std::runtime_error("d2position_dshat2 not defined for RodElement<0>");
    
    return Vec3r::Zero();
}

template<>
Mat3r RodElement<0>::orientation(Real s_hat) const
{
    // if s_hat < 0.5, position is on "rigid body" associated with first node
    // otherwise position is on "rigid body" associated with second node
    if (s_hat < 0.5)
    {
        return _nodes[0]->orientation;
    }
    else
    {
        return _nodes[1]->orientation;
    }
}

template<>
Mat3r RodElement<0>::previousOrientation(Real s_hat) const
{
    // if s_hat < 0.5, position is on "rigid body" associated with first node
    // otherwise position is on "rigid body" associated with second node
    if (s_hat < 0.5)
    {
        return _nodes[0]->prev_orientation;
    }
    else
    {
        return _nodes[1]->prev_orientation;
    }
}

template<>
Real RodElement<0>::Ni(int /* shape_func_index */, Real /* s_hat */) const
{
    // no shape functions for this type of element
    throw std::runtime_error("No shape functions defined for RodElement<0>");
}

template<>
Real RodElement<0>::dNi_dshat(int /* shape_func_index */, Real /* s_hat */ ) const
{
    // no shape functions for this type of element
    throw std::runtime_error("No shape functions defined for RodElement<0>");
}

template<>
Vec3r RodElement<0>::shearStrain(Real /* s_hat */) const
{
    Vec3r v = 0.5/_rest_length*(_nodes[0]->orientation.transpose() + _nodes[1]->orientation.transpose()) * (_nodes[1]->position - _nodes[0]->position);
    return v;
}

template<>
Vec3r RodElement<0>::bendingStrain(Real /* s_hat */) const
{
    Vec3r u = 1.0/_rest_length * Math::Minus_SO3(_nodes[1]->orientation, _nodes[0]->orientation); 
    return u;
}

template<>
Vec6r RodElement<0>::strain(Real s_hat) const
{
    Vec6r both;
    both.head<3>() = shearStrain(s_hat);
    both.tail<3>() = bendingStrain(s_hat);

    return both;
}

template<>
typename RodElement<0>::StrainGradientMatType RodElement<0>::strainGradient(Real /* s_hat */) const
{
    StrainGradientMatType grad;

    const Mat3r dCv_dp1 = -0.5*(_nodes[0]->orientation.transpose() + _nodes[1]->orientation.transpose()) / _rest_length;
    const Mat3r dCv_dp2 = -dCv_dp1;
    
    const Vec3r dp = _nodes[1]->position - _nodes[0]->position;
    const Mat3r dCv_dor1 = 0.5*Math::Skew3(_nodes[0]->orientation.transpose() * dp / _rest_length);
    const Mat3r dCv_dor2 = 0.5*Math::Skew3(_nodes[1]->orientation.transpose() * dp / _rest_length);

    const Vec3r dtheta = Math::Minus_SO3(_nodes[1]->orientation, _nodes[0]->orientation);
    const Mat3r jac_inv = Math::ExpMap_InvRightJacobian(dtheta);
    const Mat3r dCu_dor1 = -jac_inv.transpose() / _rest_length;
    const Mat3r dCu_dor2 = jac_inv / _rest_length;

    // submatrices in correct spot in overall delC matrix
    grad.block<3,3>(0, 0) = dCv_dp1;
    grad.block<3,3>(0, 3) = dCv_dor1;
    grad.block<3,3>(0, 6) = dCv_dp2;
    grad.block<3,3>(0, 9) = dCv_dor2;
    grad.block<3,3>(3, 0) = Mat3r::Zero();
    grad.block<3,3>(3, 3) = dCu_dor1;
    grad.block<3,3>(3, 6) = Mat3r::Zero();
    grad.block<3,3>(3, 9) = dCu_dor2;

    return grad;
}

template<>
Vec3r RodElement<0>::contactPoint(Real s_hat, const Vec3r& cp_local) const
{
    return position(s_hat) + orientation(s_hat) * cp_local;
}

template<>
Vec3r RodElement<0>::previousContactPoint(Real s_hat, const Vec3r& cp_local) const
{
    return previousPosition(s_hat) + previousOrientation(s_hat) * cp_local;
}

template<>
Vec3r RodElement<0>::contactPointVelocity(Real s_hat, const Vec3r& cp_local) const
{
    Vec3r stretch = 0.5*(_nodes[0]->orientation.transpose() + _nodes[1]->orientation.transpose()) * (_nodes[1]->position - _nodes[0]->position);
    Real l_c = stretch[2];

    // contact point = p1 + s_hat * l_c * R1 * e3 + R1 * cp_local

    // time derivative needs d/dt(l_c)  (i.e. the time derivative of the current element length)
    Vec3r ddt_v = 0.5 * ( _nodes[0]->orientation * Math::Skew3(_nodes[0]->ang_velocity) + _nodes[1]->orientation * Math::Skew3(_nodes[1]->ang_velocity)).transpose() * (_nodes[1]->position - _nodes[0]->position) + 
        (_nodes[0]->orientation + _nodes[1]->orientation).transpose() * (_nodes[1]->lin_velocity - _nodes[0]->lin_velocity);
    Real ddt_lc = ddt_v[2];

    if (s_hat < 0.5)
    {
        return _nodes[0]->lin_velocity + 
            s_hat * ddt_lc * _nodes[0]->orientation.col(2) + s_hat * l_c * _nodes[0]->orientation * Math::Skew3(_nodes[0]->ang_velocity).col(2) +
            _nodes[0]->orientation * Math::Skew3(_nodes[0]->ang_velocity) * cp_local;
    }
    else
    {
        return _nodes[1]->lin_velocity - 
            (1-s_hat) * ddt_lc * _nodes[1]->orientation.col(2) - s_hat * l_c * _nodes[1]->orientation * Math::Skew3(_nodes[1]->ang_velocity).col(2) +
            _nodes[1]->orientation * Math::Skew3(_nodes[1]->ang_velocity) * cp_local;
    }
}

template<>
typename RodElement<0>::ContactPointGradientMatType RodElement<0>::contactPointGradient(Real /* s_hat */, const Vec3r& /* cp_local */) const
{
    // this shouldn't be used
    throw std::runtime_error("contactPointGradient not defined for RodElement<0>");

    return ContactPointGradientMatType::Zero();
}

} // namespace SimObject