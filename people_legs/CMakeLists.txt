cmake_minimum_required(VERSION 2.8.3)
project(people_legs CXX C)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  people_msgs
  roscpp
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(people_legs include/people_legs2.cpp)

add_executable(legs src/main.cpp)

target_link_libraries(legs
   people_legs
   ${catkin_LIBRARIES}
)
