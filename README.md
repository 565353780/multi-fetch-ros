# Multi-Fetch ROS Package

## Install
```shell
sudo apt install ros-noetic-octomap libcgal-dev python3-catkin-tools
```

## Clone
```bash
mkdir -p multi_fetch_ws/src
cd multi_fetch_ws/src
git clone https://github.com/565353780/multi-fetch-ros.git
cd ..
```

## Build
```bash
cd multi_fetch_ws
catkin init
catkin config --cmake-args -DCMAKE_CXX_STANDARD=17 -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes

# if need to combine with other workspace, run this
catkin config --extend <path-to-other-ws-devel-folder>

catkin build multi-fetch-ros
```

## Run
must running this
```shell
cd multi_fetch_ws
source devel/setup.zsh
roslaunch multi_fetch_gazebo simple_start.launch
roslaunch multi_fetch_gazebo move_group.launch
```

control method 1 : use keyboard to control all fetch
```bash
rosrun multi_robot_gazebo multi_fetch_teleop_twist_keyboard.py
```

control method 2: auto nav for multi fetch
```bash
rosrun multi_fetch_controller test_multi_fetch_control_service.py
idx range : select in [0, 1, 2]
viewpoint :
input
!
to auto set viewpoint
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

