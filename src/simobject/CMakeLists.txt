cmake_minimum_required(VERSION 3.13)
project(XPBDRods)

set(SIMOBJECT_INCLUDE_DIR "../../include/simobject")
set(SIMOBJECT_SRC_DIR "./")

set(SIMOBJECT_HEADERS
    ${SIMOBJECT_INCLUDE_DIR}/rod/XPBDRod.hpp
    ${SIMOBJECT_INCLUDE_DIR}/rod/CrossSection.hpp
    ${SIMOBJECT_INCLUDE_DIR}/group/XPBDObjectGroup_Base.hpp
    ${SIMOBJECT_INCLUDE_DIR}/group/XPBDConcentricTubeRobot.hpp
)

set(SIMOBJECT_SOURCES
    ${SIMOBJECT_SRC_DIR}/OrientedParticle.cpp
    ${SIMOBJECT_SRC_DIR}/rod/XPBDRod.cpp
    ${SIMOBJECT_SRC_DIR}/rod/XPBDRodSegment.cpp
    ${SIMOBJECT_SRC_DIR}/XPBDObject_Base.cpp
    ${SIMOBJECT_SRC_DIR}/group/XPBDObjectGroup_Base.cpp
    ${SIMOBJECT_SRC_DIR}/group/XPBDConcentricTubeRobot.cpp
    ${SIMOBJECT_SRC_DIR}/group/XPBDPendulum.cpp
    ${SIMOBJECT_SRC_DIR}/rigidbody/XPBDRigidBody_Base.cpp
    ${SIMOBJECT_SRC_DIR}/rigidbody/XPBDRigidBox.cpp
    ${SIMOBJECT_SRC_DIR}/rigidbody/XPBDRigidSphere.cpp
    ${SIMOBJECT_SRC_DIR}/rigidbody/XPBDPlane.cpp
)

target_sources(SimObject
    PRIVATE
        ${SIMOBJECT_HEADERS}
        ${SIMOBJECT_SOURCES}
        
)

install(TARGETS SimObject EXPORT SimObject-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${ROD_HEADERS}
    DESTINATION include/${PROJECT_NAME}/simobject)