cmake_minimum_required(VERSION 2.8.3)
project(pose_estimation_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(pose_estimation_ros_lib src/pose_estimation_ros.cpp)

cs_add_executable(pose_estimation_ros_node src/pose_estimation_ros.cpp src/pose_estimation_ros_node.cpp)
target_link_libraries(pose_estimation_ros_node pose_estimation_ros_lib) 

cs_install()
cs_export()
