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

    virtual std::vector<ConstraintAndLambda> internalConstraintsAndLambdas() const override {}

};

} // namespace SimObject