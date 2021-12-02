# Sem-Multi-Robot Collaborative Dense Scene Reconstruction (SIGGRAPH 2019)
We provide the implementation of the optimal-mass-transport algorithm tailored for collaborative scanning along with virtual scan examples, on top of ROS.

## Requirements
This package depends on OpenCV, CGAL and OctoMap. Please install these libraries first.
```shell
sudo apt install -y ros-melodic-octomap libcgal-dev
```
or
```shell
rosdep install co_scan virtual_scan
```

If `catkin build` hasn't been installed
```shell
sudo apt install python3-catkin-tools
```

## Voxblox++ Requirements
Watch the blog for mode details
```c++
https://blog.csdn.net/qq_49466306/article/details/111872619
```

## Download and Installation
```shell
catkin_ws$ git clone https://github.com/GuoJunfu-tech/sem_co_scan.git src
catkin_ws$ mkdir -p tmp/result
catkin_ws$ catkin init
# for debug mode
catkin_ws$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug
# let cmake export compile_commands.json
catkin_ws$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
# for release mode
catkin config --cmake-args -DCMAKE_CXX_STANDARD=17 -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
# for use only
catkin config --cmake-args -DCMAKE_CXX_STANDARD=17 -DCMAKE_BUILD_TYPE=Release

catkin config --extend <path-to-Voxblox++-devel-folder>

catkin_ws$ catkin build
```

## Launch Multi Fetch (Test Passed)

### NEEDTOEDIT
```shell
navigation/base_local_planner/src/goal_functions.cpp :
  transformGlobalPlan() :
    tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);
    ->
    tf.transform(global_pose, robot_pose, plan_pose.header.frame_id, ros::Duration(0.5));
```

Run each command in different terminal
```shell
roslaunch multi_robot_gazebo multi_fetch.launch
roslaunch multi_robot_gazebo move_group.launch
rosrun multi_robot_gazebo fetch_controller.py
```

## Run
Run this with 3 different shell one by one.
Source target ros environment before run each code.
```shell
roslaunch multi_robot_gazebo simple_start.launch
roslaunch co_scan co_scan.launch

# for view_point_extractor test without visualization
# !!!now ViewPointExtractorServer will start as default, visual func closed as default
roslaunch view_point_extractor ViewPointExtractorServer.launch
rosrun view_point_extractor try_ViewPointExtractorServer
```

## Citation
If you use this code for your research, please cite our paper:
```
@article{dong2019multi,
  title={Multi-robot collaborative dense scene reconstruction},
  author={xxxx},
  journal={ACM Transactions on Graphics (TOG)},
  volume={xx},
  number={x},
  pages={xx},
  year={2021},
  publisher={ACM}
}
```
