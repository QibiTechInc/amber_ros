cmake_minimum_required(VERSION 3.0.2)
project(amber_ros_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy actionlib actionlib_msgs  industrial_trajectory_filters message_filters sensor_msgs std_msgs std_srvs message_generation
)

catkin_python_setup()

add_service_files(
  DIRECTORY srv
  FILES
  GetInt8Array.srv
  SetInt8Array.srv
  SetJointNo.srv
  SetJointTrajectory.srv
  )

add_action_files(
  DIRECTORY
  action
  FILES
  Empty.action
  )

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS rospy actionlib actionlib_msgs  industrial_trajectory_filters message_filters sensor_msgs std_msgs std_srvs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/amber_ros_driver_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

catkin_install_python(PROGRAMS
  scripts/amber_action_server_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

catkin_install_python(PROGRAMS
  scripts/amber_jointstates_integrator_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

catkin_install_python(PROGRAMS
  scripts/amber_manual_calibration
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

catkin_install_python(PROGRAMS
  scripts/amber_auto_calibration
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/amber_driver_test.py)
endif()
