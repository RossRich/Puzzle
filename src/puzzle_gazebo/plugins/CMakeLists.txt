project(PuzzlePlugins)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set (CMAKE_AUTOMOC ON)

find_package (Qt5Widgets REQUIRED)
find_package (Qt5Core REQUIRED)
find_package (Protobuf REQUIRED)
find_package (gazebo REQUIRED)

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(ModelScale SHARED ModelScale.cpp)
target_link_libraries(ModelScale
  ${GAZEBO_LIBRARIES}
)

add_library(GunPlugin SHARED GunPlugin.cpp Utils.hpp)
target_link_libraries(GunPlugin
  ${GAZEBO_LIBRARIES}
)

add_library(YPGunPlugin SHARED YPGunPlugin.cpp Utils.hpp)
target_link_libraries(YPGunPlugin
  ${GAZEBO_LIBRARIES}
)

add_library(PoseTestPlugin SHARED PoseTestPlugin.cpp Utils.hpp)
target_link_libraries(PoseTestPlugin
  ${GAZEBO_LIBRARIES}
)

# add_definitions(-DCALC_VELOCITY_ORIGINAL)
add_library(BulletPlugin SHARED BulletPlugin.cpp Utils.hpp)
target_link_libraries(BulletPlugin 
  ${GAZEBO_LIBRARIES}
)

add_library(GUIShootPlugin SHARED ${headers_MOC} GUIShootPlugin.cpp)
target_link_libraries(GUIShootPlugin 
    ${GAZEBO_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
)