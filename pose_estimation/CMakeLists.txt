cmake_minimum_required(VERSION 2.8.3)
project(pose_estimation)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
    
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(pose_estimation_lib src/pose_estimation.cpp)
target_link_libraries(pose_estimation_lib ${OpenCV_LIBS})

cs_install()
cs_export()
