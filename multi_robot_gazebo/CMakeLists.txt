cmake_minimum_required(VERSION 3.10)
project(multi_robot_gazebo)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/multi_fetch_teleop_twist_keyboard.py
  scripts/multi_fetch_tilt_head.py
  scripts/fetch_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

