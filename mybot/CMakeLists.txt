cmake_minimum_required(VERSION 2.8.3)
project(mybot)


find_package(catkin REQUIRED COMPONENTS
  gmapping
  #nox_description
  roscpp
  rospy
  teb_local_planner
  joint_state_publisher
  robot_state_publisher
  urdf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nox
#  CATKIN_DEPENDS gmapping navigation nox_description ros_control ros_controllers roscpp rospy teb_local_planner
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mybot_controller src/mybot_controller.cpp)
target_link_libraries(mybot_controller ${catkin_LIBRARIES})
