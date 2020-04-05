cmake_minimum_required(VERSION 2.8.3)
project(detector)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
    
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(detector_lib src/detector.cpp)
target_link_libraries(detector_lib ${OpenCV_LIBS})

cs_install()
cs_export()
