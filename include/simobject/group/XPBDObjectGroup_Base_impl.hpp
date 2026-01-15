#pragma once
#include "simobject/group/XPBDObjectGroup_Base.hpp"

#include "common/constraint_containers.hpp"
#include "common/object_containers.hpp"

namespace SimObject
{

struct XPBDObjectGroup_Base::Impl
{
    XPBDObjects_Container objects;
    XPBDConstraints_Container constraints;
};

template <typename ObjectType, typename... Args>
ObjectType& XPBDObjectGroup_Base::_addObject(Args&&... args)
{
    return _objects().template emplace_back<ObjectType>(std::forward<Args>(args)...);
}

template <typename ConstraintType, typename... Args>
ConstraintType& XPBDObjectGroup_Base::_addConstraint(Args&&... args)
{
    return _constraints().template emplace_back<ConstraintType>(std::forward<Args>(args)...);
}

template <typename ConstraintType, typename... Args>
ConstraintType& XPBDObjectGroup_Base::_addInternalConstraint(Args&&... args)
{
    return _internalConstraints().template emplace_back<ConstraintType>(std::forward<Args>(args)...);
}

} // namespace SimObject