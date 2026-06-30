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
    Vec3r _center;
    
    Real _wall_length;
    Real _wall_thickness;
    Real _wall_height;

    Real _gap_size;
    std::vector<bool> _gaps;

    Vec3r _wall_color;

};

} // namespace SimObject