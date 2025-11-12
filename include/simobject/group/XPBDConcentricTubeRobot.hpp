#pragma once

#include "simobject/group/XPBDObjectGroup_Base.hpp"
#include "simobject/rod/XPBDRod.hpp"

#include "config/XPBDConcentricTubeRobotConfig.hpp"

namespace SimObject
{

class XPBDConcentricTubeRobot : public XPBDObject_Base
{
public:
    XPBDConcentricTubeRobot(const Config::XPBDConcentricTubeRobotConfig& config);

    virtual void setup() override;

    virtual void internalConstraintSolve(Real dt) override;

private:
    void _updateConcentricityConstraints();

    Vec3r _base_position;
    Mat3r _base_orientation;

    // separate variables for convenience
    XPBDRod* _outer_rod;
    XPBDRod* _inner_rod;
};

} // namespace SimObject