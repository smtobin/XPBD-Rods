#pragma once

#include "config/joint/JointConfig.hpp"

namespace Config
{

class SphericalJointConfig : public JointConfig
{
public:
    explicit SphericalJointConfig()
        : JointConfig()
    {}

    explicit SphericalJointConfig(const YAML::Node& node)
        : JointConfig(node)
    {
        _extractParameter("swing-min-angle", node, _swing_min_angle);
        _extractParameter("swing-max-angle", node, _swing_max_angle);
        _extractParameter("swing-limit-compliance", node, _swing_limit_compliance);

        _extractParameter("twist-min-anlge", node, _twist_min_angle);
        _extractParameter("twist-max-angle", node, _twist_max_angle);
        _extractParameter("twist-limit-compliance", node, _twist_limit_compliance);
    }

    bool hasSwingLimit() const
    { 
        return (_swing_min_angle.value != std::numeric_limits<Real>::lowest() || _swing_max_angle.value != std::numeric_limits<Real>::max());
    }

    bool hasTwistLimit() const
    {
        return (_twist_min_angle.value != std::numeric_limits<Real>::lowest() || _twist_max_angle.value != std::numeric_limits<Real>::max());
    }

    Real swingMinAngle() const { return _swing_min_angle.value; }
    Real swingMaxAngle() const { return _swing_max_angle.value; }
    Real swingLimitCompliance() const { return _swing_limit_compliance.value; }

    Real twistMinAngle() const { return _twist_min_angle.value; }
    Real twistMaxAngle() const { return _twist_max_angle.value; }
    Real twistLimitCompliance() const { return _twist_limit_compliance.value; }

private:
    ConfigParameter<Real> _swing_min_angle = ConfigParameter<Real>(std::numeric_limits<Real>::lowest());
    ConfigParameter<Real> _swing_max_angle = ConfigParameter<Real>(std::numeric_limits<Real>::max());
    ConfigParameter<Real> _swing_limit_compliance = ConfigParameter<Real>(0);

    ConfigParameter<Real> _twist_min_angle = ConfigParameter<Real>(std::numeric_limits<Real>::lowest());
    ConfigParameter<Real> _twist_max_angle = ConfigParameter<Real>(std::numeric_limits<Real>::max());
    ConfigParameter<Real> _twist_limit_compliance = ConfigParameter<Real>(0);
};

} // namespace Config