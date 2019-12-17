cmake_minimum_required(VERSION 2.8.3)
project(bilkabot_move CXX C)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  actionlib
  move_base_msgs
)

catkin_package(
)

find_package(PkgConfig)

pkg_check_modules(GTKMM gtkmm-3.0)

link_directories(
    ${GTKMM_LIBRARY_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTKMM_INCLUDE_DIRS}
)

add_library(move include/bilkabot_move.cpp)

add_executable(move_base src/main.cpp)

target_link_libraries(move_base
  move
  ${catkin_LIBRARIES}
  ${GTKMM_LIBRARIES}
)
