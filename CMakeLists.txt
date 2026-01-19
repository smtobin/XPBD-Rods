cmake_minimum_required(VERSION 3.13)
project(XPBDRods LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)                       # specify C++ standard
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")
set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-ftime-trace)
    message(STATUS "Added -ftime-trace flag")
endif()

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

add_library(Constraint)
add_subdirectory("src/constraint")
target_link_libraries(Constraint Eigen3::Eigen)

add_library(SimObject)
add_subdirectory("src/simobject")
target_link_libraries(SimObject Eigen3::Eigen Constraint)

# add_library(RigidBody)
# add_subdirectory("src/simobject/rigidbody")

add_library(Graphics)
add_subdirectory("src/graphics")
target_link_libraries(Graphics Eigen3::Eigen ${VTK_LIBRARIES} SimObject)

add_library(Simulation)
add_subdirectory("src/simulation")
target_link_libraries(Simulation Eigen3::Eigen ${VTK_LIBRARIES} SimObject Solver Graphics yaml-cpp)


#####################################################
# SIMULATION EXECUTABLES
#####################################################
add_executable(Sim src/exec/main.cpp)
target_link_libraries(Sim Simulation ${VTK_LIBRARIES})

add_executable(RodGraspingSim src/exec/rod_grasping_sim.cpp)
target_link_libraries(RodGraspingSim Simulation ${VTK_LIBRARIES})


###################################################
# TESTS
###################################################
add_executable(TestThomas test/test_thomas.cpp)
target_link_libraries(TestThomas Solver Eigen3::Eigen)

add_executable(TestBanded test/test_banded.cpp)
target_link_libraries(TestBanded Solver Eigen3::Eigen)

add_executable(TestThomasVsBanded test/test_thomas_vs_banded.cpp)
target_link_libraries(TestThomasVsBanded Solver Eigen3::Eigen)

add_executable(TestVTK test/test_vtk.cpp)
target_link_libraries(TestVTK Eigen3::Eigen ${VTK_LIBRARIES} SimObject)

add_executable(TestPBR test/test_pbr.cpp)
target_link_libraries(TestPBR Eigen3::Eigen ${VTK_LIBRARIES})

add_executable(TestGradient test/test_gradient.cpp)
target_link_libraries(TestGradient Eigen3::Eigen Constraint SimObject)

add_executable(TestConstraintGradients test/test_constraint_gradients.cpp)
target_link_libraries(TestConstraintGradients Eigen3::Eigen Constraint SimObject)



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
  TARGETS Sim
  MODULES ${VTK_LIBRARIES}
)

vtk_module_autoinit(
  TARGETS RodGraspingSim
  MODULES ${VTK_LIBRARIES}
)

