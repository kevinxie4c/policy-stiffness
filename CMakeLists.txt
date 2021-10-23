project(policy-stiffness)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(test.out test.cpp SimCharacter.cpp)
target_link_libraries(test.out stdc++ dart assimp boost_system)
