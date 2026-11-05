#pragma once

#include "simobject/group/XPBDObjectGroup_Base.hpp"
#include "simobject/rod/XPBDHigherOrderRod.hpp"

#include "config/XPBDConcentricTubeRobotConfig.hpp"

#include <unordered_map>

namespace SimObject
{

class XPBDConcentricTubeRobot : public XPBDObjectGroup_Base
{
public:
    XPBDConcentricTubeRobot(const Config::XPBDConcentricTubeRobotConfig& config);

    virtual void setup() override;

    virtual void internalConstraintSolve(Real dt) override;

    virtual std::vector<ConstraintAndLambda> internalConstraintsAndLambdas() const override;

    virtual void velocityUpdate(Real dt) override;

    void setInnerTubeTranslation(Real it_trans);
    void setInnerTubeRotation(Real it_rot);
    void setOuterTubeTranslation(Real ot_trans);
    void setOuterTubeRotation(Real ot_rot);

private:
    void _updateConcentricityConstraints();

    // position of the "base" of the CTR
    // we define this as the exit point of the CTR (where the tubes exit the rigid sheath)
    Vec3r _base_position;
    Mat3r _base_orientation;

    /** Joint variables */
    // translation of the inner tube (0 = tip of inner tube is at the base position)
    Real _inner_tube_translation;
    // rotation of the inner tube base about the z-axis (i.e. z-axis rotation angle of inner tube relative to base orientation)
    Real _inner_tube_rotation;
    // translation of the outer tube (0 = tip of outer tube is at the base position)
    Real _outer_tube_translation;
    // rotation of the outer tube base about the z-axis (i.e. z-axis rotation angle of outer tube relative to base orientation)
    Real _outer_tube_rotation;

    Config::RodConfig _outer_tube_config;
    Config::RodConfig _inner_tube_config;

    // separate variables for convenience
    XPBDRod_<RodElement<1>>* _outer_rod;
    XPBDRod_<RodElement<1>>* _inner_rod;

    Constraint::OneSidedFixedJointConstraint* _outer_rod_base_constraint;
    Constraint::OneSidedFixedJointConstraint* _inner_rod_base_constraint;

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

    // temporary: store total t in the sim so that we can vary actuation as a function of time
    Real _t = 0;
};

} // namespace SimObject