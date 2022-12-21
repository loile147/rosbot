cmake_minimum_required(VERSION 2.8.3)
project(mybot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gmapping
  roscpp
  rospy
  joint_state_publisher
  robot_state_publisher
  urdf
  teb_local_planner
)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(mybot_controller src/mybot_controller.cpp)
target_link_libraries(mybot_controller ${catkin_LIBRARIES})
