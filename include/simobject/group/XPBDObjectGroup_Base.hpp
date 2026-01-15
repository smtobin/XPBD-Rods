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
    XPBDObjectGroup_Base(const Config::XPBDObjectConfig& config);

    virtual ~XPBDObjectGroup_Base();

    // Move operations
    XPBDObjectGroup_Base(XPBDObjectGroup_Base&&) noexcept;
    XPBDObjectGroup_Base& operator=(XPBDObjectGroup_Base&&) noexcept;
    
    // Delete copy operations
    XPBDObjectGroup_Base(const XPBDObjectGroup_Base&) = delete;
    XPBDObjectGroup_Base& operator=(const XPBDObjectGroup_Base&) = delete;

    const XPBDObjects_Container& objects() const; //{ return _objects(); }
    const XPBDConstraints_Container& constraints() const;// { return _constraints(); }

    virtual void inertialUpdate(Real dt) override;

    virtual void internalConstraintSolve(Real dt) override;

    virtual void velocityUpdate(Real dt) override;

    virtual std::vector<const OrientedParticle*> particles() const override;

protected:
    template <typename ObjectType, typename... Args>
    ObjectType& _addObject(Args&&... args);
    // {
    //     return _objects.template emplace_back<ObjectType>(std::forward<Args>(args)...);
    // }

    template <typename ConstraintType, typename... Args>
    ConstraintType& _addConstraint(Args&&... args);
    // {
    //     return _constraints.template emplace_back<ConstraintType>(std::forward<Args>(args)...);
    // }

    template <typename ConstraintType, typename... Args>
    ConstraintType& _addInternalConstraint(Args&&... args);
    // {
    //     return _internalConstraints().template emplace_back<ConstraintType>(std::forward<Args>(args)...);
    // }

protected:
    // XPBDObjects_Container _objects;
    // XPBDConstraints_Container _constraints;

    XPBDObjects_Container& _objects();
    const XPBDObjects_Container& _objects() const;

    XPBDConstraints_Container& _constraints();
    const XPBDConstraints_Container& _constraints() const;


private:
    /** Use the PIMPL idiom to reduce build time. The constraints and objects data structures are pretty heavy templates, so try
     * and move it to the .cpp file so that every file that include XPBDObject_Base doesn't have to build the templates.
     * 
     * The Impl struct has the following members:
     * - XPBDObjects_Container _objects;
     * - XPBDConstraints_Container _constraints;
     */
    struct Impl;
    std::unique_ptr<Impl> _impl;

};

} // namespace SimObject