# Multi-Fetch ROS Package

## Install
```shell
sudo apt install ros-noetic-octomap libcgal-dev python3-catkin-tools

mkdir -p multi_fetch_ws/src
cd multi_fetch_ws
catkin init
# for debug mode
catkin_ws$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug
# let cmake export compile_commands.json
catkin_ws$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
# for release mode
catkin config --cmake-args -DCMAKE_CXX_STANDARD=17 -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
# for use only
catkin config --cmake-args -DCMAKE_CXX_STANDARD=17 -DCMAKE_BUILD_TYPE=Release

# if need to combine with other workspace, run this
catkin config --extend <path-to-other-ws-devel-folder>

catkin_ws$ catkin build
```

## Run
```shell
roslaunch multi_robot_gazebo multi_fetch.launch
roslaunch multi_robot_gazebo move_group.launch

# use moveit and navigation to control all fetch
rosrun multi_robot_gazebo fetch_controller.py

# use keyboard to control all fetch
rosrun multi_robot_gazebo multi_fetch_teleop_twist_keyboard.py
```

## SOURCE EDIT HISTORY
```shell
navigation/base_local_planner/src/goal_functions.cpp :
  transformGlobalPlan() :
    tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);
    ->
    tf.transform(global_pose, robot_pose, plan_pose.header.frame_id, ros::Duration(0.5));
```

## Enjoy it~

