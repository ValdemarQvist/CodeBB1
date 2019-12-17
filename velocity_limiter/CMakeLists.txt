cmake_minimum_required(VERSION 2.8.3)
project(velocity_limiter CXX C)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
)

add_library(controller include/velocity_controller_leg.cpp)

add_executable(limiter_node src/main.cpp)

target_link_libraries(limiter_node
  controller
  ${catkin_LIBRARIES}
)
