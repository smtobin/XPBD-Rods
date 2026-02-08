#pragma once

#include <Eigen/Dense>
#include <math.h>
#include <iostream>
#include <memory>

#include "common/TypeList.hpp"
#include "common/VectorHandle.hpp"
// #include "common/VariadicVectorContainer.hpp"


/** Escape sequences to set print colors */
#define RST  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define BOLD  "\x1B[1m"
#define UNDL  "\x1B[4m"


/** Universal typedefs used by the simulation */
using Real = double;

using Vec2r = Eigen::Vector<Real, 2>;
using Vec3r = Eigen::Vector<Real, 3>;
using Vec4r = Eigen::Vector<Real, 4>;
using Vec6r = Eigen::Vector<Real, 6>;
using VecXr = Eigen::Vector<Real, -1>;

using Mat3r = Eigen::Matrix<Real, 3, 3>;
using Mat4r = Eigen::Matrix<Real, 4, 4>;
using Mat6r = Eigen::Matrix<Real, 6, 6>;
using MatXr = Eigen::Matrix<Real,-1,-1>;

/** Forward declaration of VariadicVectorContainer */
template<class L, class... R> class VariadicVectorContainer;

//////////////////////////////////////////////////////////////////////////
// Construct VariadicVectorContainer from TypeList
//////////////////////////////////////////////////////////////////////////

template<typename List>
struct VariadicVectorContainerFromTypeList;

template<typename... Types>
struct VariadicVectorContainerFromTypeList<TypeList<Types...>>
{
    using type = VariadicVectorContainer<Types...>;
    using unique_ptr_type = VariadicVectorContainer<std::unique_ptr<Types>...>;
    using ptr_type = VariadicVectorContainer<Types*...>;
    using const_ptr_type = VariadicVectorContainer<const Types*...>;
    using vector_handle_type = VariadicVectorContainer<VectorHandle<Types>...>;
    using const_vector_handle_type = VariadicVectorContainer<ConstVectorHandle<Types>...>;
};

/** Simulation object types */
namespace SimObject
{
    class XPBDObject_Base;
    class XPBDRod;
    class XPBDRodSegment;
    class XPBDRigidSphere;
    class XPBDRigidBox;
    class XPBDPlane;

    class XPBDPendulum;
    class XPBDConcentricTubeRobot;
}
using XPBDRigidBodies_TypeList = TypeList<SimObject::XPBDRigidSphere, SimObject::XPBDRigidBox, SimObject::XPBDPlane>;
/** TODO: automate this */
using XPBDRigidBodies_UniquePtrTypeList = TypeList<std::unique_ptr<SimObject::XPBDRigidSphere>, std::unique_ptr<SimObject::XPBDRigidBox>, std::unique_ptr<SimObject::XPBDPlane>>; 
using XPBDObjects_TypeList = TypeList<SimObject::XPBDRod, SimObject::XPBDRigidSphere, SimObject::XPBDRigidBox, SimObject::XPBDPlane>;
using XPBDObjects_Container = VariadicVectorContainerFromTypeList<XPBDObjects_TypeList>::type;
using XPBDObjects_UniquePtrContainer = VariadicVectorContainerFromTypeList<XPBDObjects_TypeList>::unique_ptr_type;
using XPBDObjects_PtrContainer = VariadicVectorContainerFromTypeList<XPBDObjects_TypeList>::ptr_type;
using XPBDObjects_ConstPtrContainer = VariadicVectorContainerFromTypeList<XPBDObjects_TypeList>::const_ptr_type;

using XPBDObjectGroups_TypeList = TypeList<SimObject::XPBDPendulum, SimObject::XPBDConcentricTubeRobot>;
using XPBDObjectGroups_Container = VariadicVectorContainerFromTypeList<XPBDObjectGroups_TypeList>::type;
using XPBDObjectGroups_UniquePtrContainer = VariadicVectorContainerFromTypeList<XPBDObjectGroups_TypeList>::unique_ptr_type;


/** Constraint types */
namespace Constraint
{
    class OneSidedFixedJointConstraint;
    class FixedJointConstraint;

    class OneSidedRevoluteJointConstraint;
    class NormedOneSidedRevoluteJointConstraint;
    class RevoluteJointConstraint;
    class NormedRevoluteJointConstraint;
    class RevoluteJointLimitConstraint;
    class OneSidedRevoluteJointLimitConstraint;

    class SphericalJointConstraint;
    class OneSidedSphericalJointConstraint;
    class NormedSphericalJointConstraint;
    class NormedOneSidedSphericalJointConstraint;
    class SphericalJointSwingLimitConstraint;
    class OneSidedSphericalJointSwingLimitConstraint;

    class PrismaticJointConstraint;
    class OneSidedPrismaticJointConstraint;
    class NormedPrismaticJointConstraint;
    class NormedOneSidedPrismaticJointConstraint;
    class PrismaticJointLimitConstraint;
    class OneSidedPrismaticJointLimitConstraint;

    class RodElasticConstraint;
    class PointLineConstraint;

    class RigidBodyCollisionConstraint;
    class OneSidedRigidBodyCollisionConstraint;
    class RodRigidBodyCollisionConstraint;
}

using XPBDJointConstraints_TypeList = TypeList<
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
    Constraint::NormedOneSidedPrismaticJointConstraint
>;

using XPBDJointLimitConstraints_TypeList = TypeList<
    Constraint::RevoluteJointLimitConstraint,
    Constraint::OneSidedRevoluteJointLimitConstraint,
    Constraint::SphericalJointSwingLimitConstraint,
    Constraint::OneSidedSphericalJointSwingLimitConstraint,
    Constraint::PrismaticJointLimitConstraint,
    Constraint::OneSidedPrismaticJointLimitConstraint
>;

using XPBDRodConstraints_TypeList = TypeList<
    Constraint::RodElasticConstraint,
    Constraint::PointLineConstraint
>;

using XPBDCollisionConstraints_TypeList = TypeList<
    Constraint::RigidBodyCollisionConstraint,
    Constraint::OneSidedRigidBodyCollisionConstraint,
    Constraint::RodRigidBodyCollisionConstraint
>;

using XPBDConstraints_TypeList = ConcatenateTypeLists<
    XPBDJointConstraints_TypeList,
    XPBDJointLimitConstraints_TypeList,
    XPBDRodConstraints_TypeList,
    XPBDCollisionConstraints_TypeList   // important that this goes last (I think)
>::type;

using XPBDConstraints_Container = VariadicVectorContainerFromTypeList<XPBDConstraints_TypeList>::type;
using XPBDConstraints_ConstPtrContainer = VariadicVectorContainerFromTypeList<XPBDConstraints_TypeList>::const_ptr_type;
using XPBDConstraints_ConstVectorHandleContainer = VariadicVectorContainerFromTypeList<XPBDConstraints_TypeList>::const_vector_handle_type;

using XPBDConstraints_VariantType = VariantFromTypeList<XPBDConstraints_TypeList>::variant_type;
using XPBDConstraints_ConstPtrVariantType = VariantFromTypeList<XPBDConstraints_TypeList>::const_ptr_variant_type;

using XPBDCollisionConstraints_Container = VariadicVectorContainerFromTypeList<XPBDCollisionConstraints_TypeList>::type;


/** Constraint projector types */
namespace Constraint
{
    template<typename ConstraintType>
    class XPBDConstraintProjector;

    template<typename ConstraintType>
    class XPBDSeparateConstraintProjector;

    template<typename ConstraintType>
    class Muller2020ConstraintProjector;
}

// helper struct to create container for XPBD constraint projectors
template<typename TypeList>
struct XPBDConstraintProjectorContainerFromConstraintTypeList;

template<typename ...Constraints>
struct XPBDConstraintProjectorContainerFromConstraintTypeList<TypeList<Constraints...>>
{
    using type = VariadicVectorContainer<Constraint::XPBDConstraintProjector<Constraints>...>;
};

// helper struct to create container for XPBD separate constraint projectors
template<typename TypeList>
struct XPBDSeparateConstraintProjectorContainerFromConstraintTypeList;

template<typename ...Constraints>
struct XPBDSeparateConstraintProjectorContainerFromConstraintTypeList<TypeList<Constraints...>>
{
    using type = VariadicVectorContainer<Constraint::XPBDSeparateConstraintProjector<Constraints>...>;
};

// helper struct to create container for Muller 2020 constraint projectors
template<typename TypeList>
struct Muller2020ConstraintProjectorContainerFromConstraintTypeList;

template<typename ...Constraints>
struct Muller2020ConstraintProjectorContainerFromConstraintTypeList<TypeList<Constraints...>>
{
    using type = VariadicVectorContainer<Constraint::Muller2020ConstraintProjector<Constraints>...>;
};

using XPBDConstraintProjectors_Container = XPBDConstraintProjectorContainerFromConstraintTypeList<XPBDConstraints_TypeList>::type;
using XPBDSeparateConstraintProjectors_Container = XPBDSeparateConstraintProjectorContainerFromConstraintTypeList<XPBDConstraints_TypeList>::type;
using Muller2020ConstraintProjectors_Container = Muller2020ConstraintProjectorContainerFromConstraintTypeList<XPBDConstraints_TypeList>::type;



/** Config types */ 
namespace Config
{
    class RodConfig;
    class XPBDRigidBoxConfig;
    class XPBDRigidSphereConfig;
    class XPBDPlaneConfig;
    class XPBDPendulumConfig;
    class XPBDConcentricTubeRobotConfig;

    class FixedJointConfig;
    class PrismaticJointConfig;
    class RevoluteJointConfig;
    class SphericalJointConfig;
}

using XPBDObjectConfigs_TypeList = TypeList<
    Config::RodConfig,
    Config::XPBDRigidBoxConfig,
    Config::XPBDRigidSphereConfig,
    Config::XPBDPlaneConfig,
    Config::XPBDPendulumConfig,
    Config::XPBDConcentricTubeRobotConfig
>;
using XPBDObjectConfigs_Container = VariadicVectorContainerFromTypeList<XPBDObjectConfigs_TypeList>::type;

using XPBDJointConfigs_TypeList = TypeList<
    Config::FixedJointConfig,
    Config::PrismaticJointConfig,
    Config::RevoluteJointConfig,
    Config::SphericalJointConfig
>;
using XPBDJointConfigs_Container = VariadicVectorContainerFromTypeList<XPBDJointConfigs_TypeList>::type;

/** Universal constants used by the simulation */
#define G_ACCEL 9.81    // acceleration due to gravity
#define CONSTRAINT_EPS 1e-13    // epsilon for constraints - i.e. any number less than this is treated as 0