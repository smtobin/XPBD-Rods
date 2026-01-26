#pragma once

#include "config/joint/JointConfig.hpp"

namespace Config
{

class RevoluteJointConfig : public JointConfig
{
public:
    explicit RevoluteJointConfig()
        : JointConfig()
    {}

    explicit RevoluteJointConfig(const YAML::Node& node)
        : JointConfig(node)
    {
        _extractParameter("min-angle", node, _min_angle);
        _extractParameter("max-angle", node, _max_angle);
        _extractParameter("limit-compliance", node, _limit_compliance);
    }

    bool hasAngleLimit() const
    {
        return (_min_angle.value != std::numeric_limits<Real>::lowest() || _max_angle.value != std::numeric_limits<Real>::max());
    }

    Real minAngle() const { return _min_angle.value; }
    Real maxAngle() const { return _max_angle.value; }
    Real limitCompliance() const { return _limit_compliance.value; }

private:
    ConfigParameter<Real> _min_angle = ConfigParameter<Real>(std::numeric_limits<Real>::lowest());
    ConfigParameter<Real> _max_angle = ConfigParameter<Real>(std::numeric_limits<Real>::max());
    ConfigParameter<Real> _limit_compliance = ConfigParameter<Real>(0);
};

} // namespace Config