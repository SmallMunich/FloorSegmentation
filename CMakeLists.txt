cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(floorseg)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -pthread -g -march=native")

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_library (${PROJECT_NAME}
#  CloudBin.cpp
#  CloudSegment.cpp
#  FloorSegment.cpp

#  CloudBin.h
#  CloudSegment.h
#  FloorSegment.h
#)

add_executable (floorseg test_floorseg.cpp CloudBin.cpp CloudSegment.cpp FloorSegment.cpp)
target_link_libraries (floorseg ${PCL_LIBRARIES})
