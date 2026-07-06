#pragma once

#include <Eigen/Dense>
#include <math.h>
#include <iostream>
#include <memory>
#include <cassert>

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

using Vec3i = Eigen::Vector<int, 3>;

using Mat2r = Eigen::Matrix<Real, 2, 2>;
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

/////////////////////////////////////////////////////////////////////////
// Get base type (remove keywords, references, etc.)
/////////////////////////////////////////////////////////////////////////
template<typename T>
struct base_type { using type = T; };

template<typename T>
struct base_type<T*> : base_type<T> {};

template<typename T>
struct base_type<T&> : base_type<T> {};

template<typename T>
struct base_type<T&&> : base_type<T> {};

template<typename T>
struct base_type<const T> : base_type<T> {};

template<typename T>
struct base_type<volatile T> : base_type<T> {};

template<typename T>
using base_type_t = typename base_type<T>::type;

//////////////////////////////////////////////////////////////////////////////////////

/** Simulation object types */
namespace SimObject
{
    class XPBDObject_Base;
    class XPBDRigidBody_Base;

    template<typename ElementType>
    class XPBDRod_;

    class XPBDCubicHermiteRod;

    template <int Order>
    class RodElement;
    class CubicHermiteRodElement;

    class RodCollisionSegment;
    class XPBDRigidSphere;
    class XPBDRigidBox;
    class XPBDPlane;

    class XPBDPendulum;
    class XPBDConcentricTubeRobot;
    class HexBug;
    class HexBugHabitat;
    class RPSRobot;
    class Plectoneme;
}
using XPBDRigidBodies_TypeList = TypeList<SimObject::XPBDRigidSphere, SimObject::XPBDRigidBox, SimObject::XPBDPlane>;
/** TODO: automate this */
using XPBDRigidBodies_UniquePtrTypeList = TypeList<std::unique_ptr<SimObject::XPBDRigidSphere>, std::unique_ptr<SimObject::XPBDRigidBox>, std::unique_ptr<SimObject::XPBDPlane>>; 
using XPBDObjects_TypeList = TypeList<
    SimObject::XPBDRod_<SimObject::RodElement<0>>, 
    SimObject::XPBDRod_<SimObject::RodElement<1>>, 
    SimObject::XPBDRod_<SimObject::RodElement<2>>,
    SimObject::XPBDRod_<SimObject::RodElement<3>>,
    SimObject::XPBDCubicHermiteRod,
    SimObject::XPBDRigidSphere, SimObject::XPBDRigidBox, SimObject::XPBDPlane>;
using XPBDObjects_Container = VariadicVectorContainerFromTypeList<XPBDObjects_TypeList>::type;
using XPBDObjects_UniquePtrContainer = VariadicVectorContainerFromTypeList<XPBDObjects_TypeList>::unique_ptr_type;
using XPBDObjects_PtrContainer = VariadicVectorContainerFromTypeList<XPBDObjects_TypeList>::ptr_type;
using XPBDObjects_ConstPtrContainer = VariadicVectorContainerFromTypeList<XPBDObjects_TypeList>::const_ptr_type;

using XPBDObjectGroups_TypeList = TypeList<SimObject::XPBDPendulum, SimObject::XPBDConcentricTubeRobot, SimObject::HexBug, SimObject::HexBugHabitat, SimObject::RPSRobot, SimObject::Plectoneme>;
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
    class RevoluteJointVelocityMotorConstraint;

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

    class AlignedAxesConstraint;
    class NormedAlignedAxesConstraint;
    class OneSidedAlignedAxesConstraint;
    class NormedOneSidedAlignedAxesConstraint;

    class OneSidedCoordinateConstraint;

    template <typename ElementType>
    class RodElasticGaussPointConstraint;

    class PointLineConstraint;

    class RigidBodyCollisionConstraint;
    class OneSidedRigidBodyCollisionConstraint;

    template <int Order>
    class RodRigidBodyCollisionConstraint;

    template <int Order>
    class OneSidedRodRigidBodyCollisionConstraint;

    template <int Order1, int Order2>
    class RodRodCollisionConstraint;

    class OneSidedFixedParticleConstraint;
}

using XPBDOneSidedJointConstraints_TypeList = TypeList<
    Constraint::OneSidedSphericalJointConstraint,
    Constraint::OneSidedPrismaticJointConstraint,
    Constraint::OneSidedRevoluteJointConstraint,
    Constraint::OneSidedFixedJointConstraint,
    Constraint::OneSidedAlignedAxesConstraint,
    Constraint::OneSidedCoordinateConstraint
>;

using XPBDNormedOneSidedJointConstraints_TypeList = TypeList<
    Constraint::NormedOneSidedSphericalJointConstraint,
    Constraint::NormedOneSidedPrismaticJointConstraint,
    Constraint::NormedOneSidedRevoluteJointConstraint,
    Constraint::NormedOneSidedAlignedAxesConstraint
>;

using XPBDTwoSidedJointConstraints_TypeList = TypeList<
    Constraint::FixedJointConstraint,
    Constraint::RevoluteJointConstraint,
    Constraint::SphericalJointConstraint,
    Constraint::PrismaticJointConstraint,
    Constraint::AlignedAxesConstraint
>;

using XPBDNormedTwoSidedJointConstraints_TypeList = TypeList<
    Constraint::NormedRevoluteJointConstraint,
    Constraint::NormedSphericalJointConstraint,
    Constraint::NormedPrismaticJointConstraint,
    Constraint::NormedAlignedAxesConstraint
>;

using XPBDJointVelocityMotorConstraints_TypeList = TypeList<
    Constraint::RevoluteJointVelocityMotorConstraint
>;

using XPBDJointConstraints_TypeList = ConcatenateTypeLists<
    XPBDTwoSidedJointConstraints_TypeList,
    XPBDNormedTwoSidedJointConstraints_TypeList,
    XPBDOneSidedJointConstraints_TypeList,
    XPBDNormedOneSidedJointConstraints_TypeList
>::type;

using XPBDJointLimitConstraints_TypeList = TypeList<
    Constraint::RevoluteJointLimitConstraint,
    Constraint::OneSidedRevoluteJointLimitConstraint,
    Constraint::SphericalJointSwingLimitConstraint,
    Constraint::OneSidedSphericalJointSwingLimitConstraint,
    Constraint::PrismaticJointLimitConstraint,
    Constraint::OneSidedPrismaticJointLimitConstraint
>;

using XPBDRodConstraints_TypeList = TypeList<
    Constraint::RodElasticGaussPointConstraint<SimObject::RodElement<0>>,
    Constraint::RodElasticGaussPointConstraint<SimObject::RodElement<1>>,
    Constraint::RodElasticGaussPointConstraint<SimObject::RodElement<2>>,
    Constraint::RodElasticGaussPointConstraint<SimObject::RodElement<3>>,
    Constraint::RodElasticGaussPointConstraint<SimObject::CubicHermiteRodElement>,
    Constraint::PointLineConstraint
>;

using XPBDCollisionConstraints_TypeList = TypeList<
    Constraint::RigidBodyCollisionConstraint,
    Constraint::OneSidedRigidBodyCollisionConstraint,
    Constraint::RodRigidBodyCollisionConstraint<1>,
    Constraint::RodRigidBodyCollisionConstraint<3>,
    Constraint::RodRigidBodyCollisionConstraint<2>,
    Constraint::OneSidedRodRigidBodyCollisionConstraint<1>,
    Constraint::OneSidedRodRigidBodyCollisionConstraint<2>,
    Constraint::OneSidedRodRigidBodyCollisionConstraint<3>,
    Constraint::RodRodCollisionConstraint<1,1>,
    Constraint::RodRodCollisionConstraint<1,2>,
    Constraint::RodRodCollisionConstraint<2,2>,
    Constraint::RodRodCollisionConstraint<1,3>,
    Constraint::RodRodCollisionConstraint<2,3>,
    Constraint::RodRodCollisionConstraint<3,3>
>;

using XPBDParticleConstraints_TypeList = TypeList<
    Constraint::OneSidedFixedParticleConstraint
>;

using XPBDConstraints_TypeList = ConcatenateTypeLists<
    XPBDRodConstraints_TypeList,
    XPBDParticleConstraints_TypeList,
    XPBDJointConstraints_TypeList,
    XPBDJointLimitConstraints_TypeList,
    XPBDJointVelocityMotorConstraints_TypeList,
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

    class HexBugConfig;
    class HexBugHabitatConfig;
    class RPSRobotConfig;
    class PlectonemeConfig;

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
    Config::XPBDConcentricTubeRobotConfig,
    Config::HexBugConfig,
    Config::HexBugHabitatConfig,
    Config::RPSRobotConfig,
    Config::PlectonemeConfig
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
#define COLLISION_TOL 1e-2      // if the distance between objects is less than this, register a collision and generate collision constraints
#define COLLISION_CHECK_INTERVAL 1.0/500.0 // time between collision detection
#define CONSTRAINT_EPS 1e-13    // epsilon for constraints - i.e. any number less than this is treated as 0