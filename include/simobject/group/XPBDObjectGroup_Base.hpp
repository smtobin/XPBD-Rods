#pragma once

#include "simobject/XPBDObject_Base.hpp"

#include "simobject/rigidbody/XPBDRigidBox.hpp"
#include "simobject/rigidbody/XPBDRigidSphere.hpp"
#include "simobject/rod/XPBDRod.hpp"

#include "constraint/AllConstraints.hpp"

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

    virtual void inertialUpdate(Real dt) override;

    virtual void internalConstraintSolve(Real dt) override;

    virtual void velocityUpdate(Real dt) override;

    virtual std::vector<const OrientedParticle*> particles() const override;

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

    template <typename ConstraintType, typename... Args>
    ConstraintType& _addInternalConstraint(Args&&... args)
    {
        return _internal_constraints.template emplace_back<ConstraintType>(std::forward<Args>(args)...);
    }

protected:
    XPBDObjects_Container _objects;
    XPBDConstraints_Container _constraints;

};

} // namespace SimObject