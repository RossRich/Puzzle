project(GunPlugin)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(GAZEBO)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(GunPlugin SHARED GunPlugin.cpp)
target_link_libraries(GunPlugin
  ${GAZEBO_LIBRARIES}
)