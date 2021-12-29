cmake_minimum_required(VERSION 3.0.2)
project(
  multi_fetch_controller
  LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  genmsg
  geometry_msgs
)

add_service_files(
  FILES
  ViewPointToBool.srv
)

add_message_files(
  FILES
  ViewPoint.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/multi_fetch_teleop_twist_keyboard.py
  scripts/multi_fetch_tilt_head.py
  scripts/multi_fetch_control_server.py
  scripts/test_multi_fetch_control_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

