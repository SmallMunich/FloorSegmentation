cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(gaussian_ground_segmentation)

find_package(PCL 1.7.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (gaussian_ground_segmentation test_gaussian_floor_segmentation.cpp GaussianFloorSegmentation.cpp)
target_link_libraries (gaussian_ground_segmentation ${PCL_LIBRARIES})