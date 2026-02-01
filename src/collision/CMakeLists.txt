cmake_minimum_required(VERSION 3.13)
project(XPBDRods)

set(COLLISION_INCLUDE_DIR "../../include/collision")
set(COLLISION_SRC_DIR "./")

set(COLLISION_HEADERS
    ${COLLISION_INCLUDE_DIR}/CollisionObject.hpp
    ${COLLISION_INCLUDE_DIR}/CollisionBucket.hpp
    ${COLLISION_INCLUDE_DIR}/SpatialHasher.hpp
    ${COLLISION_INCLUDE_DIR}/CollisionScene.hpp

    # SDFs
    ${COLLISION_INCLUDE_DIR}/sdf/SDF.hpp
    ${COLLISION_INCLUDE_DIR}/sdf/SphereSDF.hpp
    ${COLLISION_INCLUDE_DIR}/sdf/BoxSDF.hpp
)

set(COLLISION_SOURCES
    ${COLLISION_SRC_DIR}/CollisionObject.cpp
    ${COLLISION_SRC_DIR}/SpatialHasher.cpp
    ${COLLISION_SRC_DIR}/CollisionScene.cpp

    # SDFs
    ${COLLISION_SRC_DIR}/sdf/SphereSDF.cpp
    ${COLLISION_SRC_DIR}/sdf/BoxSDF.cpp
)

target_sources(Collision
    PRIVATE
        ${COLLISION_HEADERS}
        ${COLLISION_SOURCES}
        
)

install(TARGETS Collision EXPORT Collision-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${COLLISION_HEADERS}
    DESTINATION include/${PROJECT_NAME}/collision)