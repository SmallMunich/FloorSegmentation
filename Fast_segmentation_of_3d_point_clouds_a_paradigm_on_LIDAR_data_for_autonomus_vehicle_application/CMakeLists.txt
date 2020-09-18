cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(planefit)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -pthread -g -march=native")

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (plane_fit test_planefit.cpp GPF.cpp)
target_link_libraries (plane_fit ${PCL_LIBRARIES})
