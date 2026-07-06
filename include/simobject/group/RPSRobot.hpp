#pragma once

#include "common/common.hpp"

#include "config/RPSRobotConfig.hpp"
#include "simobject/group/XPBDObjectGroup_Base.hpp"

namespace SimObject
{

class RPSRobot : public XPBDObjectGroup_Base
{

public:
    RPSRobot(const Config::RPSRobotConfig& config);

    virtual void setup() override;

    virtual void inertialUpdate(Real dt) override;

    // virtual void internalConstraintSolve(Real dt) override;

    // virtual void velocityUpdate(Real dt) override;

    virtual std::vector<ConstraintAndLambda> internalConstraintsAndLambdas() const override { return std::vector<ConstraintAndLambda>{}; }
private:
    Constraint::OneSidedAlignedAxesConstraint* _platform_normal_constraint;
    Constraint::NormedOneSidedAlignedAxesConstraint* _normed_platform_normal_constraint;
    // Constraint::OneSidedSphericalJointConstraint* _platform_constraint;
    // Constraint::NormedOneSidedSphericalJointConstraint* _normed_platform_constraint;

    /** If true, use normed version of joint constraints.  */
    bool _normed_constraints;

    Vec3r _initial_position;
    Real _theta = 0;

};

} // namespace SimObject