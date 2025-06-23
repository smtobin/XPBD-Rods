cmake_minimum_required(VERSION 3.13)
project(XPBDRods)

set(SIMULATION_INCLUDE_DIR "../../include/simulation")
set(SIMULATION_SRC_DIR "./")

set(SIMULATION_HEADERS
    ${SIMULATION_INCLUDE_DIR}/Simulation.hpp
)

set(SIMULATION_SOURCES
    ${SIMULATION_SRC_DIR}/Simulation.cpp
)

target_sources(Simulation
    PRIVATE
        ${SIMULATION_HEADERS}
        ${SIMULATION_SOURCES}
        
)

install(TARGETS Simulation EXPORT Simulation-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${SIMULATION_HEADERS}
    DESTINATION include/${PROJECT_NAME}/simulation)