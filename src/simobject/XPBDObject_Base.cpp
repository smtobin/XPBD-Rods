#include "simobject/XPBDObject_Base.hpp"
#include "common/constraint_containers.hpp"

namespace SimObject
{

XPBDObject_Base::XPBDObject_Base(const Config::XPBDObjectConfig& config)
    : _name(config.name()), _mu_s(config.staticFrictionCoeff()), _mu_d(config.dynamicFrictionCoeff()), _collisions(config.collisions())
{
}

} // namespace SimObject