#pragma once

#include "common/common.hpp"
#include <iostream>


class Math
{

public:

static Mat3r Skew3(const Vec3r& vec)
{
    Mat3r mat;
    mat << 0,       -vec(2),    vec(1),
           vec(2),  0,          -vec(0),
           -vec(1), vec(0),     0;
    return mat;
}

static Vec3r Vee3(const Mat3r& mat)
{
    return Vec3r(mat(2,1), mat(0,2), mat(1,0));
}

// Computes right Jacobian of SO(3) exponential map
static Mat3r ExpMap_RightJacobian(const Vec3r& theta)
{
    Real theta_norm = theta.norm();
    const Mat3r skew = Skew3(theta);
    if (theta_norm < Real(1e-8))
    {
        return Mat3r::Identity() - 0.5 * skew;
    }
    
    return Mat3r::Identity() - (1 - std::cos(theta_norm)) / (theta_norm * theta_norm) * skew + (theta_norm - std::sin(theta_norm)) / (theta_norm * theta_norm * theta_norm) * skew * skew;
}

// Computes the inverse of the right Jacobian of SO(3) exponential map
static Mat3r ExpMap_InvRightJacobian(const Vec3r& theta)
{
    Real theta_norm = theta.norm();
    const Mat3r skew = Skew3(theta);

    if (theta_norm < Real(1e-8))
    {
        return Mat3r::Identity() + 0.5 * skew;
    }

    return Mat3r::Identity() + 0.5*skew + (1/(theta_norm*theta_norm) - (1+std::cos(theta_norm)) / (2*theta_norm*std::sin(theta_norm)) ) * skew * skew;
}

static Mat3r Exp_so3(const Vec3r& vec)
{
    const Mat3r skew = Skew3(vec);
    Real mag = vec.norm();

    if (mag < Real(1e-8))
        return Mat3r::Identity() + skew;
    
    return Mat3r::Identity() + std::sin(mag) / mag * skew + (1 - std::cos(mag)) / (mag * mag) * skew * skew;
}

static Vec3r Log_SO3(const Mat3r& mat)
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

static Vec3r Minus_SO3(const Mat3r& mat1, const Mat3r& mat2)
{
    return Log_SO3(mat2.transpose() * mat1);
}

static Mat3r Plus_SO3(const Mat3r& SO3_mat, const Vec3r& so3_vec)
{
    return SO3_mat * Exp_so3(so3_vec);
}

static Mat3r RotMatFromXYZEulerAngles(const Vec3r& euler_xyz)
{
    const Real x = euler_xyz(0) * M_PI / 180.0;
    const Real y = euler_xyz(1) * M_PI / 180.0;
    const Real z = euler_xyz(2) * M_PI / 180.0;

    // using the "123" convention: rotate first about x axis, then about y, then about z
    Mat3r rot_mat;
    rot_mat(0,0) = std::cos(y) * std::cos(z);
    rot_mat(0,1) = std::sin(x)*std::sin(y)*std::cos(z) - std::cos(x)*std::sin(z);
    rot_mat(0,2) = std::cos(x)*std::sin(y)*std::cos(z) + std::sin(x)*std::sin(z);

    rot_mat(1,0) = std::cos(y)*std::sin(z);
    rot_mat(1,1) = std::sin(x)*std::sin(y)*std::sin(z) + std::cos(x)*std::cos(z);
    rot_mat(1,2) = std::cos(x)*std::sin(y)*std::sin(z) - std::sin(x)*std::cos(z);

    rot_mat(2,0) = -std::sin(y);
    rot_mat(2,1) = std::sin(x)*std::cos(y);
    rot_mat(2,2) = std::cos(x)*std::cos(y);

    return rot_mat;
}

/** Projects a point p onto the line segment defined by ab.
 * Returns the interpolation factor - i.e. if in [0,1] the projected point is between a and b.
 * To get the projected point: p_proj = a + projectPointOnLine(p, a, b) * (b-a);
 */
static Real projectPointOntoLine(const Vec3r& p, const Vec3r& a, const Vec3r& b)
{
    return (p-a).dot(b-a) / (b-a).squaredNorm();
}

/** Find closest points on two line segments defined by (p1, p2) and (p3, p4) */
static std::pair<Real, Real> findClosestPointsOnLineSegments(const Vec3r& p1, const Vec3r& p2, const Vec3r& p3, const Vec3r& p4)
{
    const Vec3r d1 = p2 - p1;
    const Vec3r d2 = p4 - p3;
    const Vec3r w = p1 - p3;

    Real a = d1.dot(d1);
    Real b = d1.dot(d2);
    Real c = d2.dot(d2);
    Real d = w.dot(d1);
    Real e = w.dot(d2);
    
    Real den = a*c - b*b;

    Real beta1, beta2;
    if (den < 1e-8) // line segments are roughly parallel
    {
        Real best_dist_sq = std::numeric_limits<Real>::max();

        // project p1 onto (p3,p4)
        Real beta2_1 = std::clamp(projectPointOntoLine(p1, p3, p4), 0.0, 1.0);
        Real dist_sq1 = ((1-beta2_1)*p3 + beta2_1*p4 - p1).squaredNorm();
        best_dist_sq = dist_sq1;
        beta1 = 0.0;
        beta2 = beta2_1;

        // project p2 onto (p3, p4)
        Real beta2_2 = std::clamp(projectPointOntoLine(p2, p3, p4), 0.0, 1.0);
        Real dist_sq2 = ((1-beta2_2)*p3 + beta2_2*p4 - p2).squaredNorm();
        if (dist_sq2 < best_dist_sq)
        {
            best_dist_sq = dist_sq2;
            beta1 = 1.0;
            beta2 = beta2_2;
        }

        // project p3 onto (p1, p2)
        Real beta1_1 = std::clamp(projectPointOntoLine(p3, p1, p2), 0.0, 1.0);
        Real dist_sq3 = ((1-beta1_1)*p1 + beta1_1*p2 - p3).squaredNorm();
        if (dist_sq3 < best_dist_sq)
        {
            best_dist_sq = dist_sq3;
            beta1 = beta1_1;
            beta2 = 0.0;
        }

        // project p4 onto (p1, p2)
        Real beta1_2 = std::clamp(projectPointOntoLine(p4, p1, p2), 0.0, 1.0);
        Real dist_sq4 = ((1-beta1_2)*p1 + beta1_2*p2 - p4).squaredNorm();
        if (dist_sq4 < best_dist_sq)
        {
            best_dist_sq = dist_sq4;
            beta1 = beta1_2;
            beta2 = 1.0;
        }
    }
    else
    {
        beta1 = (b*e - c*d) / den;
        beta2 = (a*e - b*d) / den;

        if (beta1 < 0 || beta1 > 1)
        {
            // if beta1 was clamped, re-project to find beta2
            beta1 = std::clamp(beta1, 0.0, 1.0);
            beta2 = std::clamp(projectPointOntoLine((1-beta1)*p1 + beta1*p2, p3, p4), 0.0, 1.0);
            
        }
        else if (beta2 < 0 || beta2 > 1)
        {
            // if beta2 was clamped, re-project to find beta1
            beta2 = std::clamp(beta2, 0.0, 1.0);
            beta1 = std::clamp(projectPointOntoLine((1-beta2)*p3 + beta2*p4, p1, p2), 0.0, 1.0);
        }
    }

    return std::make_pair(beta1, beta2);
}

};