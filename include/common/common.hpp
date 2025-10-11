#include <Eigen/Dense>
#include <math.h>
#include <iostream>

#include "common/TypeList.hpp"
#include "common/VariadicVectorContainer.hpp"


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

namespace SimObject
{
    class XPBDRod;
    class XPBDRigidSphere;
    class XPBDRigidBox;
    class XPBDPendulum;
    class XPBDFourBarLinkage;
}
using XPBDObjects_TypeList = TypeList<SimObject::XPBDRod, SimObject::XPBDRigidSphere, SimObject::XPBDRigidBox>;
using XPBDObjects_Container = VariadicVectorContainerFromTypeList<XPBDObjects_TypeList>::type;
using XPBDObject_UniquePtrContainer = VariadicVectorContainerFromTypeList<XPBDObjects_TypeList>::unique_ptr_type;

using XPBDObjectGroups_TypeList = TypeList<SimObject::XPBDPendulum, SimObject::XPBDFourBarLinkage>;
using XPBDObjectGroups_Container = VariadicVectorContainerFromTypeList<XPBDObjectGroups_TypeList>::type;
using XPBDObjectGroups_UniquePtrContrinaer = VariadicVectorContainerFromTypeList<XPBDObjectGroups_TypeList>::unique_ptr_type;

namespace Constraint
{
    class FixedJointConstraint;
    class RodElasticConstraint;
    class RevoluteJointConstraint;
}
using XPBDConstraints_TypeList = TypeList<Constraint::FixedJointConstraint, Constraint::RodElasticConstraint, Constraint::RevoluteJointConstraint>;
using XPBDConstraints_Container = VariadicVectorContainerFromTypeList<XPBDConstraints_TypeList>::type;
using XPBDConstraints_ConstPtrContainer = VariadicVectorContainerFromTypeList<XPBDConstraints_TypeList>::const_ptr_type;


namespace Config
{
    class RodConfig;
    class XPBDRigidBoxConfig;
    class XPBDRigidSphereConfig;
    class XPBDPendulumConfig;
}

using XPBDObjectConfigs_TypeList = TypeList<
    Config::RodConfig,
    Config::XPBDRigidBoxConfig,
    Config::XPBDRigidSphereConfig,
    Config::XPBDPendulumConfig
>;
using XPBDObjectConfigs_Container = VariadicVectorContainerFromTypeList<XPBDObjectConfigs_TypeList>::type;

/** Universal constants used by the simulation */
#define G_ACCEL 9.81    // acceleration due to gravity