cmake_minimum_required(VERSION 3.13)
project(XPBDRods LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)                       # specify C++ standard
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")
set(CMAKE_BUILD_TYPE Debug)

find_package(Eigen3 3.3 REQUIRED)   # find Eigen

include_directories("./include")

add_library(Solver)
add_subdirectory("src/solver")
target_link_libraries(Solver Eigen3::Eigen)

add_executable(TestThomas test/test_thomas.cpp)
target_link_libraries(TestThomas Solver Eigen3::Eigen)

