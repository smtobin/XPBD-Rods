#include "simobject/group/XPBDObjectGroup_Base.hpp"

namespace SimObject
{
void XPBDObjectGroup_Base::inertialUpdate(Real dt)
{
    _objects.for_each_element([&](auto& obj){
        obj.inertialUpdate(dt);
    });
}

void XPBDObjectGroup_Base::internalConstraintSolve(Real dt)
{
    _objects.for_each_element([&](auto& obj){
        obj.internalConstraintSolve(dt);
    });
}

void XPBDObjectGroup_Base::velocityUpdate(Real dt)
{
    _objects.for_each_element([&](auto& obj){
        obj.velocityUpdate(dt);
    });
}

} // namespace SimObject