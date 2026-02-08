cmake_minimum_required(VERSION 3.13)
project(XPBDRods)

set(CONSTRAINT_INCLUDE_DIR "../../include/constraint")
set(CONSTRAINT_SRC_DIR "./")

set(CONSTRAINT_HEADERS
    ${CONSTRAINT_INCLUDE_DIR}/Constraint.hpp
    ${CONSTRAINT_INCLUDE_DIR}/RodElasticConstraint.hpp
    ${CONSTRAINT_INCLUDE_DIR}/FixedJointConstraint.hpp
    ${CONSTRAINT_INCLUDE_DIR}/RevoluteJointConstraint.hpp
    ${CONSTRAINT_INCLUDE_DIR}/SphericalJointConstraint.hpp
    ${CONSTRAINT_INCLUDE_DIR}/PrismaticJointConstraint.hpp
    ${CONSTRAINT_INCLUDE_DIR}/RevoluteJointLimitConstraint.hpp
    ${CONSTRAINT_INCLUDE_DIR}/SphericalJointSwingLimitConstraint.hpp
    ${CONSTRAINT_INCLUDE_DIR}/PrismaticJointLimitConstraint.hpp
    ${CONSTRAINT_INCLUDE_DIR}/PointLineConstraint.hpp

    ${CONSTRAINT_INCLUDE_DIR}/RigidBodyCollisionConstraint.hpp
    ${CONSTRAINT_INCLUDE_DIR}/RodRigidBodyCollisionConstraint.hpp
)

set(CONSTRAINT_SOURCES
    ${CONSTRAINT_SRC_DIR}/RodElasticConstraint.cpp
    ${CONSTRAINT_SRC_DIR}/FixedJointConstraint.cpp
    ${CONSTRAINT_SRC_DIR}/RevoluteJointConstraint.cpp
    ${CONSTRAINT_SRC_DIR}/SphericalJointConstraint.cpp
    ${CONSTRAINT_SRC_DIR}/PrismaticJointConstraint.cpp
    ${CONSTRAINT_SRC_DIR}/RevoluteJointLimitConstraint.cpp
    ${CONSTRAINT_SRC_DIR}/SphericalJointSwingLimitConstraint.cpp
    ${CONSTRAINT_SRC_DIR}/PrismaticJointLimitConstraint.cpp

    ${CONSTRAINT_SRC_DIR}/PointLineConstraint.cpp

    ${CONSTRAINT_SRC_DIR}/RigidBodyCollisionConstraint.cpp
    ${CONSTRAINT_SRC_DIR}/RodRigidBodyCollisionConstraint.cpp
)

target_sources(Constraint
    PRIVATE
        ${CONSTRAINT_HEADERS}
        ${CONSTRAINT_SOURCES}
        
)

install(TARGETS Constraint EXPORT Constraint-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${CONSTRAINT_HEADERS}
    DESTINATION include/${PROJECT_NAME}/constraint)