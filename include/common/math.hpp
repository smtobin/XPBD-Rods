#ifndef __MATH_HPP
#define __MATH_HPP

#include "common/common.hpp"
#include <iostream>


Mat3r Skew3(const Vec3r& vec)
{
    Mat3r mat;
    mat << 0,       -vec(2),    vec(1),
           vec(2),  0,          -vec(0),
           -vec(1), vec(0),     0;
    return mat;
}

Vec3r Vee3(const Mat3r& mat)
{
    return Vec3r(mat(2,1), mat(0,2), mat(1,0));
}

// Computes Jacobian of SO(3) exponential map
Mat3r ExpMap_Jacobian(const Vec3r& theta)
{
    Real theta_norm = theta.norm();
    const Mat3r skew = Skew3(theta);
    if (theta_norm < Real(1e-8))
    {
        return Mat3r::Identity() + 0.5 * skew;
    }
    
    return Mat3r::Identity() + (1 - std::cos(theta_norm)) / (theta_norm * theta_norm) * skew + (theta_norm - std::sin(theta_norm)) / (theta_norm * theta_norm * theta_norm) * skew * skew;
}

Mat3r Exp_so3(const Vec3r& vec)
{
    const Mat3r skew = Skew3(vec);
    Real mag = vec.norm();

    if (mag < Real(1e-8))
        return Mat3r::Identity() + skew;
    
    return Mat3r::Identity() + std::sin(mag) / mag * skew + (1 - std::cos(mag)) / (mag * mag) * skew * skew;
}

Vec3r Log_SO3(const Mat3r& mat)
{
    // std::cout << "\n===Log_SO3===" << std::endl;
    // std::cout << "  mat:\n" << mat << std::endl;
    // std::cout << "  mat.trace()-3: " << mat.trace()-3 << std::endl;
    Real theta = std::acos( std::min(0.5 * mat.trace() - 0.5, Real(1.0)));  // make sure 1/2 tr(mat) - 1/2 is not >1, will get NaNs. This may happen due to numerical drift
    // std::cout << "  theta: " << theta << std::endl;

    if (std::abs(theta) < Real(1e-14))
    {
        return Vec3r::Zero();
    }

    const Vec3r skew_vec3 = Vee3(mat - mat.transpose());
    // std::cout << "  skew_vec3: " << skew_vec3 << std::endl;

    if (std::abs(mat.trace()) < Real(1e-8))
    {
        return 0.5 * (1 + theta*theta/6.0 + 7*theta*theta*theta*theta/360.0) * skew_vec3;
    }

    // std::cout << " 2*std::sin(theta): " << 2*std::sin(theta) << std::endl;
    return theta / ( 2*std::sin(theta)) * skew_vec3;
}

Vec3r Minus_SO3(const Mat3r& mat1, const Mat3r& mat2)
{
    return Log_SO3(mat2.transpose() * mat1);
}

Mat3r Plus_SO3(const Mat3r& SO3_mat, const Vec3r& so3_vec)
{
    return SO3_mat * Exp_so3(so3_vec);
}

#endif // __MATH_HPP