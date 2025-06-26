cmake_minimum_required(VERSION 3.13)
project(XPBDRods LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)                       # specify C++ standard
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")
set(CMAKE_BUILD_TYPE Debug)

find_package(Eigen3 3.3 REQUIRED)   # find Eigen
find_package(VTK COMPONENTS
    CommonColor
    CommonCore
    FiltersSources
    IOImage
    InteractionStyle
    RenderingContextOpenGL2
    RenderingCore
    RenderingFreeType
    RenderingGL2PSOpenGL2
    RenderingOpenGL2
)

if (NOT VTK_FOUND)
  message(FATAL_ERROR "Unable to find the VTK build folder.")
endif()

include_directories("./include")

add_library(Solver)
add_subdirectory("src/solver")
target_link_libraries(Solver Eigen3::Eigen)

add_library(Rod)
add_subdirectory("src/rod")
target_link_libraries(Rod Eigen3::Eigen)

add_library(Graphics)
add_subdirectory("src/graphics")
target_link_libraries(Graphics Eigen3::Eigen ${VTK_LIBRARIES} Rod)

add_library(Simulation)
add_subdirectory("src/simulation")
target_link_libraries(Simulation Eigen3::Eigen ${VTK_LIBRARIES} Rod Solver Graphics)

add_executable(TestThomas test/test_thomas.cpp)
target_link_libraries(TestThomas Solver Eigen3::Eigen)

add_executable(TestVTK test/test_vtk.cpp)
target_link_libraries(TestVTK Eigen3::Eigen ${VTK_LIBRARIES} Rod)

add_executable(TestPBR test/test_pbr.cpp)
target_link_libraries(TestPBR Eigen3::Eigen ${VTK_LIBRARIES})

add_executable(RunSim src/exec/main.cpp)
target_link_libraries(RunSim Simulation ${VTK_LIBRARIES})

# vtk_module_autoinit is needed
vtk_module_autoinit(
  TARGETS TestVTK
  MODULES ${VTK_LIBRARIES}
)

vtk_module_autoinit(
  TARGETS TestPBR
  MODULES ${VTK_LIBRARIES}
)

vtk_module_autoinit(
  TARGETS RunSim
  MODULES ${VTK_LIBRARIES}
)

