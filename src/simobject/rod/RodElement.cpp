#include "simobject/rod/RodElement.hpp"

namespace SimObject
{

Real linearN1(Real s_hat) { return -s_hat + 1; }
Real linearN2(Real s_hat) { return s_hat; }
Real dlinearN1(Real /* s_hat */) { return -1; }
Real dlinearN2(Real /* s_hat */) { return 1; }
Real d2linearN1(Real /* s_hat */) { return 0; }
Real d2linearN2(Real /* s_hat */) { return 0; }

Real quadraticN1(Real s_hat) { return 2*s_hat*s_hat - 3*s_hat + 1; }
Real quadraticN2(Real s_hat) { return -4*s_hat*s_hat + 4*s_hat; }
Real quadraticN3(Real s_hat) { return 2*s_hat*s_hat - s_hat; }
Real dquadraticN1(Real s_hat) { return 4*s_hat - 3; }
Real dquadraticN2(Real s_hat) { return -8*s_hat + 4; }
Real dquadraticN3(Real s_hat) { return 4*s_hat - 1; }
Real d2quadraticN1(Real /* s_hat */) { return 4; }
Real d2quadraticN2(Real /* s_hat */) { return -8; }
Real d2quadraticN3(Real /* s_hat */) { return 4; }

template<int Order>
RodElement<Order>::RodElement(const NodeArrayType& nodes_list, Real rest_length)
    : _nodes(nodes_list), _bases{}, _bases_derivatives{}, _rest_length(rest_length)
{
    // get pointers to basis functions and their derivatives, according to element order
    if constexpr (Order == 1)
    {
        _bases = {linearN1, linearN2};
        _bases_derivatives = {dlinearN1, dlinearN2};
        _bases_derivatives2 = {d2linearN1, d2linearN2};
    }
    else if constexpr (Order == 2)
    {
        _bases = {quadraticN1, quadraticN2, quadraticN3};
        _bases_derivatives = {dquadraticN1, dquadraticN2, dquadraticN3};
        _bases_derivatives2 = {d2quadraticN1, d2quadraticN2, d2quadraticN3};
    }
    else
    {
        static_assert(0);
    }
}

template<int Order>
std::array<Real, Order+1> RodElement<Order>::lumpedMasses()
{
    if constexpr (Order == 1)
    {
        return {0.5, 0.5};
    }
    else if constexpr (Order == 2)
    {
        return {1.0/6.0, 2.0/3.0, 1.0/6.0};
    }
    else
    {
        static_assert(0);
        return std::array<Real, Order+1>{};
    }
}

template<int Order>
Vec3r RodElement<Order>::position(Real s_hat) const
{
    Vec3r p = _bases[0](s_hat) * _nodes[0]->position;
    for (int i = 1; i < Order+1; i++)
    {
        p += _bases[i](s_hat) * _nodes[i]->position;
    }

    return p;
}

template<int Order>
Vec3r RodElement<Order>::dposition_dshat(Real s_hat) const
{
    Vec3r dp = _bases_derivatives[0](s_hat) * _nodes[0]->position;
    for (int i = 1; i < Order+1; i++)
    {
        dp += _bases_derivatives[i](s_hat) * _nodes[i]->position;
    }

    return dp;
}

template<int Order>
Vec3r RodElement<Order>::d2position_dshat2(Real s_hat) const
{
    Vec3r d2p = _bases_derivatives2[0](s_hat) * _nodes[0]->position;
    for (int i = 1; i < Order+1; i++)
    {
        d2p += _bases_derivatives2[i](s_hat) * _nodes[i]->position;
    }

    return d2p;
}

template <int Order>
Mat3r RodElement<Order>::orientation(Real s_hat) const
{
    // get interpolated relative rotation vector
    Vec3r theta = Vec3r::Zero();
    for (int i = 1; i < Order+1; i++)
    {
        theta += _bases[i](s_hat) * Math::Minus_SO3(_nodes[i]->orientation, _nodes[0]->orientation);
    }

    // use exponential map to get interpolated rotation
    return Math::Plus_SO3(_nodes[0]->orientation, theta);
}

template <int Order>
Real RodElement<Order>::Ni(int shape_func_index, Real s_hat) const
{
    return _bases[shape_func_index](s_hat);
}

template <int Order>
Real RodElement<Order>::dNi_dshat(int shape_func_index, Real s_hat) const
{
    return _bases_derivatives[shape_func_index](s_hat);
}

template <int Order>
Vec3r RodElement<Order>::shearStrain(Real s_hat) const
{
    Mat3r R = orientation(s_hat);

    Vec3r dp_ds = Vec3r::Zero();
    for (int i = 0; i < Order+1; i++)
    {
        dp_ds += _bases_derivatives[i](s_hat) * _nodes[i]->position;
    }

    return R.transpose() * dshat_ds() * dp_ds - Vec3r(0,0,1);
}

template <int Order>
Vec3r RodElement<Order>::bendingStrain(Real s_hat) const
{
    Vec3r theta = Vec3r::Zero();
    Vec3r dtheta_ds = Vec3r::Zero();
    for (int i = 1; i < Order+1; i++)
    {
        theta += _bases[i](s_hat) * Math::Minus_SO3(_nodes[i]->orientation, _nodes[0]->orientation);
        dtheta_ds += dshat_ds() * _bases_derivatives[i](s_hat) * Math::Minus_SO3(_nodes[i]->orientation, _nodes[0]->orientation);
    }

    Vec3r u1 = Math::ExpMap_RightJacobian(theta) * dtheta_ds;

    return u1;
}

template <int Order>
Vec6r RodElement<Order>::strain(Real s_hat) const
{
    Vec6r both;
    both.head<3>() = shearStrain(s_hat);
    both.tail<3>() = bendingStrain(s_hat);

    return both;
}

template <int Order>
typename RodElement<Order>::StrainGradientMatType RodElement<Order>::strainGradient(Real s_hat) const
{
    StrainGradientMatType grad;
    
    // stores (Ri boxminus R1) for i = 1,...,Order+1
    std::array<Vec3r, Order+1> Ri_minus_R1;
    // stores Gamma^{-1} (Ri boxminus R1) for i = 1,...,Order+1
    std::array<Mat3r, Order+1> gam_inv_Ri_minus_R1;
    // stores d theta(s) / d Ri for i = 1,...,Order+1
    std::array<Mat3r, Order+1> dtheta_dRi;
    // stores d/d Ri ( d theta(s) / ds) for i = 1,...Order+1
    std::array<Mat3r, Order+1> dtheta_ds_dRi;

    // theta(s)
    Vec3r theta = Vec3r::Zero();
    Vec3r dtheta_ds = Vec3r::Zero();

    // precompute quantities
    dtheta_dRi[0] = Mat3r::Zero();
    dtheta_ds_dRi[0] = Mat3r::Zero();
    for (int i = 1; i < Order+1; i++)
    {
        Ri_minus_R1[i] = Math::Minus_SO3(_nodes[i]->orientation, _nodes[0]->orientation);
        gam_inv_Ri_minus_R1[i] = Math::ExpMap_InvRightJacobian(Ri_minus_R1[i]);
        dtheta_dRi[i] = _bases[i](s_hat) * gam_inv_Ri_minus_R1[i];
        dtheta_ds_dRi[i] = dshat_ds() * _bases_derivatives[i](s_hat) * gam_inv_Ri_minus_R1[i];

        theta += _bases[i](s_hat) * Ri_minus_R1[i];
        dtheta_ds += dshat_ds() * _bases_derivatives[i](s_hat) * Ri_minus_R1[i];

        // add contribution to dtheta_dR0 and dtheta_ds_dR0
        dtheta_dRi[0] -= dtheta_dRi[i].transpose();
        dtheta_ds_dRi[0] -= dtheta_ds_dRi[i].transpose();
    }


    /** Compute gradients of shear strain */

    // precompute useful quantities for gradients of shear strain
    Mat3r exp_theta = Math::Exp_so3(theta);
    Mat3r gam_theta = Math::ExpMap_RightJacobian(theta);
    Mat3r R = _nodes[0]->orientation * exp_theta;


    Vec3r dp_ds = Vec3r::Zero();
    for (int i = 0; i < Order+1; i++)
    {
        dp_ds += dshat_ds() * _bases_derivatives[i](s_hat) * _nodes[i]->position;
    }
    Mat3r RT_dp_ds_R = R.transpose() * Math::Skew3(dp_ds) * R;

    // gradients of shear strain
    for (int i = 0; i < Order+1; i++)
    {
        grad.template block<3,3>(0,6*i) = R.transpose() * dshat_ds() * _bases_derivatives[i](s_hat);

        if (i == 0)
        {
            grad.template block<3,3>(0,6*i+3) = RT_dp_ds_R * (exp_theta.transpose() + gam_theta * dtheta_dRi[i]);
        }
        else
        {
            grad.template block<3,3>(0,6*i+3) = RT_dp_ds_R * gam_theta * dtheta_dRi[i];
        }
    }


    /** Compute gradients of bending strain */

    // gradients of bending strain
    for (int i = 0; i < Order+1; i++)
    {
        // gradient w.r.t. positions = 0
        grad.template block<3,3>(3, 6*i) = Mat3r::Zero();

        // gradient w.r.t. orientation
        grad.template block<3,3>(3, 6*i+3) = Math::DExpMap_RightJacobian_Contract_j(theta, dtheta_ds) * dtheta_dRi[i] + gam_theta * dtheta_ds_dRi[i];    
    }

    return grad;
}

 /** Finds closest points between two rod elements. Use Newton's method to solve the optimization problem that minimizes squared error. */
template <int Order1, int Order2>
void closestPointsBetweenRodElements(const SimObject::RodElement<Order1>* elem1, const SimObject::RodElement<Order2>* elem2)
{
    // get initial estimates based on linear approximation
    std::array<std::pair<Real, Real>, Order1*Order2> seeds;
    for (int i = 0; i < Order1; i++)
    {
        for (int j = 0; j < Order2; j++)
        {
            auto [s1_loc, s2_loc] = Math::findClosestPointsOnLineSegments(
                elem1->nodes()[i]->position, elem1->nodes()[i+1]->position, elem2->nodes()[j]->position, elem2->nodes()[j+1]->position
            );
            Real s1 = s1_loc / Order1 + Real(i) / Order1;
            Real s2 = s2_loc / Order2 + Real(j) / Order2;

            seeds[i*Order1 + j] = std::make_pair(s1, s2);

            std::cout << "starting s1 and s2: " << s1 << ", " << s2 << std::endl;
        }
    }

    for (const auto& seed : seeds)
    {
        Real s1 = seed.first;
        Real s2 = seed.second;

        int num_iters = 10;
        // Newton's method
        // for (int iter = 0; iter < num_iters; iter++)
        // {
        //     Vec3r p1 = elem1->position(s1);
        //     Vec3r dp1 = elem1->dposition_dshat(s1);
        //     Vec3r d2p1 = elem1->d2position_dshat2(s1);
        //     Vec3r p2 = elem2->position(s2);
        //     Vec3r dp2 = elem2->dposition_dshat(s2);
        //     Vec3r d2p2 = elem2->d2position_dshat2(s2);

        //     Vec3r diff = p1 - p2;

        //     Mat2r J;
        //     J(0,0) = 2*dp1.dot(dp1) + 2*diff.dot(d2p1);
        //     J(0,1) = -2*dp1.dot(dp2);
        //     J(1,0) = J(0,1);
        //     J(1,1) = 2*dp2.dot(dp2) - 2*diff.dot(d2p2);

        //     Vec2r res;
        //     res[0] = -2*diff.dot(dp1);
        //     res[1] = 2*diff.dot(dp2);

        //     Real det = J(0,0)*J(1,1) - J(0,1)*J(1,0);

        //     if (det < 1e-12)
        //         break;
            
        //     Vec2r ds = Vec2r(
        //         J(1,1)*res(0) - J(0,1)*res(1),
        //         -J(1,0)*res(0) + J(0,0)*res(1)
        //     ) / det;

        //     std::cout << "ds: " << ds.transpose() << std::endl;

        //     s1 = std::clamp(s1 + ds[0], 0.0, 1.0);
        //     s2 = std::clamp(s2 + ds[1], 0.0, 1.0);
        // }

        /** TODO: try Gauss-Newton
         * 
         * f(x) = ||r(x)||^2
         * 
         * (J^T * J) dx = -J^T * r
         * 
         * where J is the jacobian of r, the residual vector (p1 - p2)
         */

        // coordinate descent
        for (int iter = 0; iter < num_iters; iter++)
        {
            // solve for s1 with s2 fixed
            Vec3r p1 = elem1->position(s1);
            Vec3r dp1 = elem1->dposition_dshat(s1);
            Vec3r d2p1 = elem1->d2position_dshat2(s1);

            Vec3r p2 = elem2->position(s2);

            Real g1 = (p1 - p2).dot(dp1);
            Real dg1 = dp1.dot(dp1) + (p1 - p2).dot(d2p1);
            if (std::abs(dg1) > 1e-12)
                s1 = std::clamp(s1 - g1/dg1, Real(0.0), Real(1.0));

            // solve for s2 with s1 fixed
            p1 = elem1->position(s1);
            Vec3r dp2 = elem2->dposition_dshat(s2);
            Vec3r d2p2 = elem2->d2position_dshat2(s2);

            Real g2 = (p1 - p2).dot(dp2);
            Real dg2 = -dp2.dot(dp2) + (p1 - p2).dot(d2p2);
            if (std::abs(dg2) > 1e-12)
                s2 = std::clamp(s2 - g2/dg2, Real(0.0), Real(1.0));

            std::cout << "New s1, s2: " << s1 << ", " << s2 << std::endl;
        }

        std::cout << "\nFinal s1 and s2: " << s1 << ", " << s2 << std::endl;
        std::cout << "Distance: " << (elem1->position(s1) - elem2->position(s2)).norm() << std::endl;
    }

    


}   

template class RodElement<1>;
template class RodElement<2>;

template void closestPointsBetweenRodElements<1,1>(
    const RodElement<1>*, const RodElement<1>*);

template void closestPointsBetweenRodElements<1,2>(
    const RodElement<1>*, const RodElement<2>*);

template void closestPointsBetweenRodElements<2,1>(
    const RodElement<2>*, const RodElement<1>*);

template void closestPointsBetweenRodElements<2,2>(
    const RodElement<2>*, const RodElement<2>*);

} // namespace SimObject