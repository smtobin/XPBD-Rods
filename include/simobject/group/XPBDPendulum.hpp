#pragma once

#include "simobject/group/XPBDObjectGroup_Base.hpp"
#include "simobject/rigidbody/XPBDRigidBox.hpp"

#include "config/XPBDPendulumConfig.hpp"

#include "constraint/RevoluteJointConstraint.hpp"

namespace SimObject
{

class XPBDPendulum : public XPBDObjectGroup_Base
{

public:
    XPBDPendulum(const Config::XPBDPendulumConfig& config);

    virtual ~XPBDPendulum();

    // Move operations
    XPBDPendulum(XPBDPendulum&&) noexcept;
    XPBDPendulum& operator=(XPBDPendulum&&) noexcept;
    
    // Delete copy operations
    XPBDPendulum(const XPBDPendulum&) = delete;
    XPBDPendulum& operator=(const XPBDPendulum&) = delete;

    virtual void setup() override;

private:
    int _num_bodies;
    Real _initial_angle;

};

} // namespace SimObject