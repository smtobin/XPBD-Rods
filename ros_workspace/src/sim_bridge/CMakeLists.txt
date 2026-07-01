cmake_minimum_required(VERSION 3.8)
project(sim_bridge)

# specify C++ standard - C++17 is used mainly for template metaprogramming stuff
set(CMAKE_CXX_STANDARD 17)
# use 03 optimizer, turn on all warnings
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")
# by default, use Debug build so we can use gdb easily
set(CMAKE_BUILD_TYPE RelWithDebInfo)

# Set RPATH to find local dependencies
# set(CMAKE_SKIP_BUILD_RPATH FALSE)
# set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
# set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(SIM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../..")

# Explicitly add your dependency library paths
# set(CMAKE_INSTALL_RPATH 
#     "${SIM_DIR}/build"
# )

# set(CMAKE_BUILD_RPATH 
#     "${SIM_DIR}/build"
# )

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

# find assimp
find_package(assimp REQUIRED)
find_package(yaml-cpp REQUIRED)

# find_package(XPBDRods REQUIRED)

add_subdirectory(${SIM_DIR} XPBDRods)
include_directories("${SIM_DIR}/include")

# === Create Node Executable ===
add_executable(${PROJECT_NAME}_node 
  src/sim_bridge_node.cpp 
  src/SimBridge.cpp
)

# ROS dependencies
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs geometry_msgs)

target_link_libraries(${PROJECT_NAME}_node
  Common
  Collision
  Solver
  Constraint
  SimObject
  Graphics
  Simulation
  assimp
  Eigen3::Eigen
  yaml-cpp::yaml-cpp
  ${VTK_LIBRARIES}
)

vtk_module_autoinit(
  TARGETS ${PROJECT_NAME}_node
  MODULES ${VTK_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
