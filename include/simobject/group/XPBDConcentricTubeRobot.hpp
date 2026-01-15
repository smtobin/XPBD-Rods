#pragma once

#include "simobject/group/XPBDObjectGroup_Base.hpp"
#include "simobject/rod/XPBDRod.hpp"

#include "config/XPBDConcentricTubeRobotConfig.hpp"

#include <unordered_map>

namespace SimObject
{

class XPBDConcentricTubeRobot : public XPBDObjectGroup_Base
{
public:
    XPBDConcentricTubeRobot(const Config::XPBDConcentricTubeRobotConfig& config);

    virtual ~XPBDConcentricTubeRobot();

    // Move operations
    XPBDConcentricTubeRobot(XPBDConcentricTubeRobot&&) noexcept;
    XPBDConcentricTubeRobot& operator=(XPBDConcentricTubeRobot&&) noexcept;
    
    // Delete copy operations
    XPBDConcentricTubeRobot(const XPBDConcentricTubeRobot&) = delete;
    XPBDConcentricTubeRobot& operator=(const XPBDConcentricTubeRobot&) = delete;

    virtual void setup() override;

    virtual void internalConstraintSolve(Real dt) override;

    virtual std::vector<ConstraintAndLambda> internalConstraintsAndLambdas() const override;

private:
    void _updateConcentricityConstraints();

    Vec3r _base_position;
    Mat3r _base_orientation;

    // separate variables for convenience
    XPBDRod* _outer_rod;
    XPBDRod* _inner_rod;

    // index all the particles
    std::unordered_map<const OrientedParticle*, int> _particle_ptr_to_index;

    // global delC matrix
    VecXr _M_inv;
    VecXr _alpha;
    MatXr _delC_mat;
    MatXr _LHS_mat;
    VecXr _RHS_vec;
    VecXr _dlam;
    VecXr _dx;
};

} // namespace SimObject