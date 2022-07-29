cmake_minimum_required(VERSION 3.0.2)
project(amber_simple_joint_command_sender)

find_package(catkin REQUIRED COMPONENTS
  rospy control_msgs std_msgs message_generation
)

add_message_files(
  DIRECTORY
  msg
  FILES
  SimpleJointTrajectory.msg
  )

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS rospy control_msgs std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/amber_simple_joint_command_sender_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/amber_simple_joint_command_sender_node
  )
