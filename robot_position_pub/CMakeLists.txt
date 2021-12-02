cmake_minimum_required(VERSION 3.0.2)
project(robot_position_pub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_filters
  tf
)

catkin_package()

add_executable(${PROJECT_NAME}_node
  src/RobotPositionPub.cpp
  src/RobotPositionPub_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node PRIVATE ${catkin_LIBRARIES})

target_include_directories(${PROJECT_NAME}_node PRIVATE include ${catkin_INCLUDE_DIRS})

target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)
