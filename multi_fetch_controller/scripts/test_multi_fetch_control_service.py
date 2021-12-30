#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from multi_fetch_controller.msg import ViewPoint
from multi_fetch_controller.srv import ViewPointToBool

if __name__ == "__main__":
    robot_idx = input("Please input robot idx :")

    rospy.init_node("test_fetch_" + robot_idx + "_control_service")

    fetch_service_name = "/fetch_" + robot_idx + "_control_service"

    position = input("Please input position like 'x,y,z' :")
    if position == "!":
        if robot_idx == "0":
            position_x = 1
            position_y = -6
            position_z = 1
            direction_x = 1
            direction_y = -1
            direction_z = 0
        elif robot_idx == "2":
            position_x = -1
            position_y = -7
            position_z = 1
            direction_x = -1
            direction_y = -1
            direction_z = 0
        elif robot_idx == "1":
            position_x = -2
            position_y = 5
            position_z = 1
            direction_x = -1
            direction_y = 0
            direction_z = 0
    else:
        position_split_list = position.split(",")
        position_x = float(position_split_list[0])
        position_y = float(position_split_list[1])
        position_z = float(position_split_list[2])

        direction = input("Please input direction like 'x,y' :")
        direction_split_list = direction.split(",")
        direction_x = float(direction_split_list[0])
        direction_y = float(direction_split_list[1])
        direction_z = float(direction_split_list[2])

    print("start moveToViewPoint :")
    print("\t position = [", position_x, position_y, position_z, "]")
    print("\t direction = [", direction_x, direction_y, direction_z, "]")

    try:
        fetch_move_serve = rospy.ServiceProxy(fetch_service_name, ViewPointToBool)
        viewpoint = ViewPoint()
        viewpoint.position.x = position_x
        viewpoint.position.y = position_y
        viewpoint.position.z = position_z
        viewpoint.direction.x = direction_x
        viewpoint.direction.y = direction_y
        viewpoint.direction.z = direction_z
        is_success = fetch_move_serve(viewpoint)
    except rospy.ServiceException as e:
        print("Service call failed:", e)

