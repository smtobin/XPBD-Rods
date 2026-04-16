#pragma once

#include "simobject/group/XPBDObjectGroup_Base.hpp"
#include "simobject/rod/XPBDHigherOrderRod.hpp"

#include "config/HexBugConfig.hpp"

#include <unordered_map>

namespace SimObject
{

class HexBug : public XPBDObjectGroup_Base
{
public:
    HexBug(const Config::HexBugConfig& config);

    virtual void setup() override;

    // virtual void internalConstraintSolve(Real dt) override;

    virtual void velocityUpdate(Real dt) override;

    virtual std::vector<ConstraintAndLambda> internalConstraintsAndLambdas() const override {}

private:
    Vec3r _body_initial_position;
    Vec3r _body_size;
    Real _body_density;

    Real _motor_angular_velocity;

    Real _leg_stiffness;
    Real _leg_length;
    Real _leg_length_increment;
    Real _leg_diameter;
    Vec3r _leg_curvature;

};

} // namespace SimObject