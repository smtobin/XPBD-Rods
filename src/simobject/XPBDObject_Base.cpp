#include "simobject/XPBDObject_Base.hpp"
#include "common/constraint_containers.hpp"

namespace SimObject
{

struct XPBDObject_Base::Impl
{
    XPBDConstraints_Container internal_constraints;
    VecXr internal_lambda;
};

XPBDObject_Base::XPBDObject_Base(const Config::XPBDObjectConfig& config)
    : _name(config.name()), _impl(std::make_unique<Impl>())
{

}

XPBDObject_Base::~XPBDObject_Base() = default;
XPBDObject_Base::XPBDObject_Base(XPBDObject_Base&&) noexcept = default;
XPBDObject_Base& XPBDObject_Base::operator=(XPBDObject_Base&&) noexcept = default;

XPBDConstraints_Container& XPBDObject_Base::_internalConstraints()
{
    return _impl->internal_constraints;
}

const XPBDConstraints_Container& XPBDObject_Base::_internalConstraints() const
{
    return _impl->internal_constraints;
}

VecXr& XPBDObject_Base::_internalLambda()
{
    return _impl->internal_lambda;
}

const VecXr& XPBDObject_Base::_internalLambda() const
{
    return _impl->internal_lambda;
}

} // namespace SimObject