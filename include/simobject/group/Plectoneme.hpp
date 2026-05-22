#pragma once

#include "simobject/group/XPBDObjectGroup_Base.hpp"
#include "simobject/rod/XPBDHigherOrderRod.hpp"

#include "config/PlectonemeConfig.hpp"

#include <unordered_map>
#include <variant>

namespace SimObject
{

class Plectoneme : public XPBDObjectGroup_Base
{
public:
    Plectoneme(const Config::PlectonemeConfig& config);

    virtual void setup() override;

    // virtual void internalConstraintSolve(Real dt) override;

    virtual void velocityUpdate(Real dt) override;

    virtual std::vector<ConstraintAndLambda> internalConstraintsAndLambdas() const override {}

private:
    std::variant<XPBDRod_<RodElement<1>>*, XPBDRod_<RodElement<2>>*, XPBDRod_<RodElement<3>>*> _rod;

    Real _cur_twist;
    Real _max_twist;

    Real _cur_displacement;
    Real _max_displacement;

};

} // namespace SimObject