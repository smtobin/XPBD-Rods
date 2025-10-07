#pragma once

#include "simobject/XPBDObject_Base.hpp"

#include <vector>
#include <memory>

namespace SimObject
{

class XPBDObjectGroup_Base : public XPBDObject_Base
{
public:
    XPBDObjectGroup_Base(const Config::XPBDObjectConfig& config)
        : XPBDObject_Base(config)
    {
    }

    const XPBDObjects_Container& objects() const { return _objects; }
    const XPBDConstraints_Container& constraints() const { return _constraints; }

    virtual void inertialUpdate(Real dt) override
    {
        _objects.for_each_element([&](auto& obj){
            obj.inertialUpdate(dt);
        });
    }

    virtual void internalConstraintSolve(Real dt) override
    {
        _objects.for_each_element([&](auto& obj){
            obj.internalConstraintSolve(dt);
        });
    }

    virtual void velocityUpdate(Real dt) override
    {
        _objects.for_each_element([&](auto& obj){
            obj.velocityUpdate(dt);
        });
    }

protected:
    template <typename ObjectType, typename... Args>
    ObjectType& _addObject(Args&&... args)
    {
        return _objects.template emplace_back<ObjectType>(std::forward<Args>(args)...);
    }

    template <typename ConstraintType, typename... Args>
    ConstraintType& _addConstraint(Args&&... args)
    {
        return _constraints.template emplace_back<ConstraintType>(std::forward<Args>(args)...);
    }

protected:
    XPBDObjects_Container _objects;
    XPBDConstraints_Container _constraints;

};

} // namespace SimObject