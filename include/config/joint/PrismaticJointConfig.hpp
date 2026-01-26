#pragma once

#include "config/joint/JointConfig.hpp"

namespace Config
{

class PrismaticJointConfig : public JointConfig
{
public:
    explicit PrismaticJointConfig()
        : JointConfig()
    {}

    explicit PrismaticJointConfig(const YAML::Node& node)
        : JointConfig(node)
    {
        _extractParameter("min-translation", node, _min_trans);
        _extractParameter("max-translation", node, _max_trans);
        _extractParameter("limit-compliance", node, _limit_compliance);
    }

    bool hasTranslationLimit() const
    {
        return (_min_trans.value != std::numeric_limits<Real>::lowest() || _max_trans.value != std::numeric_limits<Real>::max());
    }

    Real minTranslation() const { return _min_trans.value; }
    Real maxTranslation() const { return _max_trans.value; }
    Real limitCompliance() const { return _limit_compliance.value; }

private:
    ConfigParameter<Real> _min_trans = ConfigParameter<Real>(std::numeric_limits<Real>::lowest());
    ConfigParameter<Real> _max_trans = ConfigParameter<Real>(std::numeric_limits<Real>::max());
    ConfigParameter<Real> _limit_compliance = ConfigParameter<Real>(0);
};

} // namespace Config