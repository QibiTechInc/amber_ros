cmake_minimum_required(VERSION 3.0.2)
project(amber_gazebo)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  gazebo_ros
  gazebo_ros_control
  urdf
  xacro
)

catkin_package(
   CATKIN_DEPENDS controller_manager gazebo_ros gazebo_ros_control urdf xacro
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY
  config
  launch
  world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
