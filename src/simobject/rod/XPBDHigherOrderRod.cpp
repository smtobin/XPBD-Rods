#include "simobject/rod/XPBDHigherOrderRod.hpp"

namespace SimObject
{

template <int Order>
XPBDRod_<Order>::XPBDRod_(const Config::RodConfig& config)
    : XPBDObject_Base(config),
    _length(config.length()), _base_fixed(config.baseFixed()), _tip_fixed(config.tipFixed()),
    _density(config.density()), _E(config.E()), _nu(config.nu()),
    _solver(Order, config.nodes())
{
    // compute shear modulus
    _G = _E / (2 * (1+_nu));
}

} // namespace SimObject