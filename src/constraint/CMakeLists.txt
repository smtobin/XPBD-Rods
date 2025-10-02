cmake_minimum_required(VERSION 3.13)
project(XPBDRods)

set(CONSTRAINT_INCLUDE_DIR "../../include/constraint")
set(CONSTRAINT_SRC_DIR "./")

set(CONSTRAINT_HEADERS
    ${CONSTRAINT_INCLUDE_DIR}/Constraint.hpp
    ${CONSTRAINT_INCLUDE_DIR}/RodElasticConstraint.hpp
    ${CONSTRAINT_INCLUDE_DIR}/AttachmentConstraint.hpp
    ${CONSTRAINT_INCLUDE_DIR}/PointLineConstraint.hpp
)

set(CONSTRAINT_SOURCES
    ${CONSTRAINT_SRC_DIR}/RodElasticConstraint.cpp
    ${CONSTRAINT_SRC_DIR}/AttachmentConstraint.cpp
    ${CONSTRAINT_SRC_DIR}/PointLineConstraint.cpp
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