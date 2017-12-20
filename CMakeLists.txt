cmake_minimum_required(VERSION 2.8.3)
project(px4_experiment)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  nav_msgs
  roscpp
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rotate_once_z src/rotate_once_z.cpp)
add_executable(2d_movement_test src/2d_movement_test.cpp)

target_link_libraries(rotate_once_z ${catkin_LIBRARIES})
target_link_libraries(2d_movement_test ${catkin_LIBRARIES})
