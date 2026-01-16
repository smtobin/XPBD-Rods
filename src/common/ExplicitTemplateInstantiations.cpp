/**
 * Explicit template instantiations for heavy container types.
 * This prevents multiple instantiations across translation units,
 * which is a major source of compilation time overhead.
 */

#include "common/common.hpp"
#include "common/VariadicVectorContainer.hpp"
#include "constraint/AllConstraints.hpp"

#include "simobject/rigidbody/XPBDRigidBox.hpp"
#include "simobject/rigidbody/XPBDRigidSphere.hpp"
#include "simobject/rod/XPBDRod.hpp"

#include "simobject/group/XPBDConcentricTubeRobot.hpp"
#include "simobject/group/XPBDPendulum.hpp"

// Explicitly instantiate the massive container types
// These are only instantiated once here, not in every translation unit that includes them

// XPBDObjects_Container instantiation
template class VariadicVectorContainer<
    SimObject::XPBDRod,
    SimObject::XPBDRigidSphere,
    SimObject::XPBDRigidBox
>;

// XPBDObject_UniquePtrContainer instantiation  
template class VariadicVectorContainer<
    std::unique_ptr<SimObject::XPBDRod>,
    std::unique_ptr<SimObject::XPBDRigidSphere>,
    std::unique_ptr<SimObject::XPBDRigidBox>
>;

// XPBDObjectGroups_Container instantiation
template class VariadicVectorContainer<
    SimObject::XPBDPendulum,
    SimObject::XPBDConcentricTubeRobot
>;

// XPBDObjectGroups_UniquePtrContainer instantiation
template class VariadicVectorContainer<
    std::unique_ptr<SimObject::XPBDPendulum>,
    std::unique_ptr<SimObject::XPBDConcentricTubeRobot>
>;

// XPBDConstraints_Container instantiation (16 types!)
template class VariadicVectorContainer<
    Constraint::OneSidedFixedJointConstraint,
    Constraint::FixedJointConstraint,
    Constraint::OneSidedRevoluteJointConstraint,
    Constraint::NormedOneSidedRevoluteJointConstraint,
    Constraint::RevoluteJointConstraint,
    Constraint::NormedRevoluteJointConstraint,
    Constraint::SphericalJointConstraint,
    Constraint::OneSidedSphericalJointConstraint,
    Constraint::NormedSphericalJointConstraint,
    Constraint::NormedOneSidedSphericalJointConstraint,
    Constraint::PrismaticJointConstraint,
    Constraint::OneSidedPrismaticJointConstraint,
    Constraint::NormedPrismaticJointConstraint,
    Constraint::NormedOneSidedPrismaticJointConstraint,
    Constraint::RodElasticConstraint,
    Constraint::PointLineConstraint
>;

// XPBDConstraints_ConstPtrContainer instantiation
template class VariadicVectorContainer<
    const Constraint::OneSidedFixedJointConstraint*,
    const Constraint::FixedJointConstraint*,
    const Constraint::OneSidedRevoluteJointConstraint*,
    const Constraint::NormedOneSidedRevoluteJointConstraint*,
    const Constraint::RevoluteJointConstraint*,
    const Constraint::NormedRevoluteJointConstraint*,
    const Constraint::SphericalJointConstraint*,
    const Constraint::OneSidedSphericalJointConstraint*,
    const Constraint::NormedSphericalJointConstraint*,
    const Constraint::NormedOneSidedSphericalJointConstraint*,
    const Constraint::PrismaticJointConstraint*,
    const Constraint::OneSidedPrismaticJointConstraint*,
    const Constraint::NormedPrismaticJointConstraint*,
    const Constraint::NormedOneSidedPrismaticJointConstraint*,
    const Constraint::RodElasticConstraint*,
    const Constraint::PointLineConstraint*
>;
