cmake_minimum_required(VERSION 2.8.3)
project(detector_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(detector_ros_lib src/detector_ros.cpp)

cs_add_executable(detector_ros_node src/detector_ros.cpp src/detector_ros_node.cpp)
target_link_libraries(detector_ros_node detector_ros_lib) 

cs_install()
cs_export()
