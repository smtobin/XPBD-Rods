#pragma once

#include "simobject/group/XPBDObjectGroup_Base.hpp"

#include "config/HexBugHabitatConfig.hpp"

#include <unordered_map>

namespace SimObject
{

class HexBugHabitat : public XPBDObjectGroup_Base
{
public:
    HexBugHabitat(const Config::HexBugHabitatConfig& config);

    virtual void setup() override;

    virtual std::vector<ConstraintAndLambda> internalConstraintsAndLambdas() const override {}

private:
    Vec3r _wall_size;
    Vec3r _wall_color;

};

} // namespace SimObject