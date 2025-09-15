cmake_minimum_required(VERSION 3.13)
project(XPBDRods)

set(ROD_INCLUDE_DIR "../../../include/simobject/rod")
set(ROD_SRC_DIR "./")

set(ROD_HEADERS
    ${ROD_INCLUDE_DIR}/XPBDRod.hpp
    ${ROD_INCLUDE_DIR}/CrossSection.hpp
)

set(ROD_SOURCES
    ${ROD_SRC_DIR}/XPBDRod.cpp
)

target_sources(Rod
    PRIVATE
        ${ROD_HEADERS}
        ${ROD_SOURCES}
        
)

install(TARGETS Rod EXPORT Rod-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${ROD_HEADERS}
    DESTINATION include/${PROJECT_NAME}/rod)