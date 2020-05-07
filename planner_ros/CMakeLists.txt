cmake_minimum_required(VERSION 2.8.3)
project(planner_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(planner_ros_lib src/planner_ros.cpp)

cs_add_executable(planner_ros_node src/planner_ros.cpp src/planner_ros_node.cpp)
target_link_libraries(planner_ros_node planner_ros_lib) 

cs_install()
cs_export()
