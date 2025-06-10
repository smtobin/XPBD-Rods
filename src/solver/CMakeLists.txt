cmake_minimum_required(VERSION 3.13)
project(XPBDRods)

set(SOLVER_INCLUDE_DIR "../../include/solver")
set(SOLVER_SRC_DIR "./")

set(SOLVER_HEADERS
    ${SOLVER_INCLUDE_DIR}/BlockThomasSolver.hpp
)

set(SOLVER_SOURCES
    ${SOLVER_SRC_DIR}/BlockThomasSolver.cpp
)

target_sources(Solver
    PRIVATE
        ${SOLVER_HEADERS}
        ${SOLVER_SOURCES}
        
)

install(TARGETS Solver EXPORT Solver-targets
    RUNTIME DESTINATION bin/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES ${SOLVER_HEADERS}
    DESTINATION include/${PROJECT_NAME}/solver)