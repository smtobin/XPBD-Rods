cmake_minimum_required(VERSION 3.13)
project(XPBDRods)

set(COMMON_INCLUDE_DIR "../../include/common")
set(COMMON_SRC_DIR "./")

set(COMMON_HEADERS
    ${COMMON_INCLUDE_DIR}/Mesh.hpp
)

set(COMMON_SOURCES
    ${COMMON_SRC_DIR}/Mesh.cpp
)

target_sources(Common
    PRIVATE
        ${COMMON_HEADERS}
        ${COMMON_SOURCES}
        
)

install(TARGETS Common EXPORT Common-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${COMMON_HEADERS}
    DESTINATION include/${PROJECT_NAME}/graphics)