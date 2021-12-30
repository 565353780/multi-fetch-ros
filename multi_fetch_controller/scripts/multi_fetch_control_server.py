#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    scripts.multi_fetch_controller
    ~~~~~~~~~~~~~~~~~~~~

    A class FetchController, implementing a valid method to move fetch robot to the target viewpoint.

    :copyright: (c) 2021.12.20 by chLi.
    :license: MIT License, see LICENSE for more details.
"""

from math import sqrt

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist

from multi_fetch_controller.srv import ViewPointToBool

class FetchController(object):
    """ Control the camera by controlling fetch """
    def __init__(self, robot_idx, end_effector_frame='wrist_roll_link', group_name='arm_with_torso'):
        ''' Init of FetchController
        Args:
            robot_idx: str
            end_effector_frame: str
            group_name: str | select in ["arm", "arm_with_torso"]
        Return:
            None
        '''
        self.robot_idx = robot_idx
        self.end_effector_frame = end_effector_frame
        self.group_name = group_name

        # init common params
        self.joint_names = ["torso_lift_joint", "shoulder_pan_joint",
                            "shoulder_lift_joint", "upperarm_roll_joint",
                            "elbow_flex_joint", "forearm_roll_joint",
                            "wrist_flex_joint", "wrist_roll_joint"]
        self.shrink_arm_pose = [0, 2, 1.6, 0, 1.57, 0, 1.8, 0]
        self.extend_arm_pose = [0, -0.595508, -0.914319, 1.15083, 1.52896, 0.688713, -0.660242, -1.23424]
        self.directly_moving_speed = 0.5
        self.rate = 50
        self.dist_between_base_and_camera = 0.5
        self.max_waiting_times = 180
        self.retry_times = 10

        self.move_base_goal = MoveBaseGoal()
        self.move_base_goal.target_pose.header.frame_id = 'map'

        self.end_effector_pose_stamped = PoseStamped()
        self.end_effector_pose_stamped.header.frame_id = 'fetch_' + self.robot_idx + '/base_link'

        self.vel_cmd = Twist()
        self.vel_cmd.linear.y = 0
        self.vel_cmd.linear.z = 0
        self.vel_cmd.angular.x = 0
        self.vel_cmd.angular.y = 0
        self.vel_cmd.angular.z = 0
        rospy.loginfo("Init Common Params Succeeded!")

        # init all services
        self.planning_scene = PlanningSceneInterface(
            "base_link",
            "fetch_" + self.robot_idx)
        self.planning_scene.removeCollisionObject("my_front_ground")
        self.planning_scene.removeCollisionObject("my_back_ground")
        self.planning_scene.removeCollisionObject("my_right_ground")
        self.planning_scene.removeCollisionObject("my_left_ground")
        self.planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
        self.planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
        self.planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
        self.planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

        self.move_base_client = actionlib.SimpleActionClient(
            "fetch_" + self.robot_idx + "/move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait_result = self.move_base_client.wait_for_server()
        if wait_result:
            rospy.loginfo("Connected to move base server")
        else:
            rospy.loginfo("Can not connect to move base server")
            exit()

        self.move_group = MoveGroupInterface(
            self.group_name, "fetch_" + self.robot_idx + "/base_link",
            move_group="fetch_" + self.robot_idx + "/move_group")

        self.cmd_vel_pub = rospy.Publisher('fetch_' + self.robot_idx + '/cmd_vel', Twist, queue_size=1)

        rospy.sleep(1.0)
        rospy.loginfo("Init All Services Succeeded!")
        return

    def compute_base_target_pose(self, camera_position, camera_direction, safe_dist):
        """ Compute the fetch base(_link) target pose of the move we need
        Args:
            camera_position: list/tuple [x, y, z]
            camera_direction: list/tuple [x, y, z]
            safe_dist: float
        Return:
            base_target_pose: Pose
        """
        xy_plane_dist = sqrt(camera_direction[0] ** 2 + camera_direction[1] ** 2)
        if xy_plane_dist == 0:
            xy_plane_dist = 1

        xy_plane_vector = [camera_direction[0] / xy_plane_dist, camera_direction[1] / xy_plane_dist]

        return Pose(Point(camera_position[0] - (self.dist_between_base_and_camera + safe_dist) * xy_plane_vector[0],
                          camera_position[1] - (self.dist_between_base_and_camera + safe_dist) * xy_plane_vector[1],
                          0),
                    self.compute_quaternion((xy_plane_vector[0], xy_plane_vector[1], 0)))

    def compute_end_effector_relative_pose(self, camera_position, camera_direction):
        """ Compute the pose of the end effector relative to the robot base, the end effector frame may be different from camera frame.
            However, moveit can only control the pose of the end effector, so we have to infer the end effector pose by camera pose
        Args:
            camera_position: list/tuple [x, y, z]
            camera_direction: list/tuple [x, y, z]
        Return:
            relative_pose: Pose
        """
        xy_plane_dist = sqrt(camera_direction[0] ** 2 + camera_direction[1] ** 2)
        if xy_plane_dist == 0:
            xy_plane_dist = 1

        xy_plane_vector = [camera_direction[0] / xy_plane_dist, camera_direction[1] / xy_plane_dist]

        relative_position = Point(self.dist_between_base_and_camera, 0, camera_position[2])
        relative_direction = [camera_direction[0] * xy_plane_vector[0] + camera_direction[1] * xy_plane_vector[1],
                              camera_direction[1] * xy_plane_vector[0] - camera_direction[0] * xy_plane_vector[1],
                              camera_direction[2]]

        return Pose(relative_position, self.compute_quaternion(relative_direction))

    def compute_quaternion(self, face_to_direction):
        """ Compute the Quaternion of the move we need
        Args:
            face_to_direction: list/tuple [x, y, z]
        Return:
            Quaternion object (x, y, z, w)
        """
        point_2d_dist2 = face_to_direction[0] **2 + face_to_direction[1] **2
        xy = sqrt(point_2d_dist2)

        point_3d_dist2 = point_2d_dist2 + face_to_direction[2] **2
        xyz = sqrt(point_3d_dist2)

        if xyz == 0:
            return Quaternion(0, 0, 0, 1)
        if xy == 0:
            if face_to_direction[2] > 0:
                return Quaternion(0, -1 / sqrt(2), 0, 1 / sqrt(2))
            return Quaternion(0, 1 / sqrt(2), 0, 1 / sqrt(2))

        cos_fai = xy / xyz

        cos_theta = face_to_direction[0] / xy
        sin_theta = face_to_direction[1] / xy

        cos_half_fai = sqrt((1 + cos_fai) / 2)
        if face_to_direction[2] > 0:
            sin_half_fai = sqrt((1 - cos_fai) / 2)
        else:
            sin_half_fai = - sqrt((1 - cos_fai) / 2)

        cos_half_theta = sqrt((1 + cos_theta) / 2)
        if face_to_direction[1] > 0:
            sin_half_theta = sqrt((1 - cos_theta) / 2)
        else:
            sin_half_theta = - sqrt((1 - cos_theta) / 2)

        return Quaternion(sin_half_fai * (cos_half_theta * sin_theta - sin_half_theta * cos_theta),
                          sin_half_fai * (sin_half_theta * sin_theta - cos_half_theta * cos_theta),
                          cos_half_fai * sin_half_theta, cos_half_fai * cos_half_theta)

    def control_arm_by_moveit(self, moveit_method, target_pose):
        ''' Control Arm by MoveIt 
        Args:
            moveit_method: str | select in ["moveToJointPosition", "moveToPose"]
            target_pose: list/tuple | target arm pose
        Return:
            bool | success
        '''
        rospy.loginfo("Start moving wrist...")

        run_times = 0
        while run_times < self.retry_times:
            if rospy.is_shutdown():
                return False

            if moveit_method == "moveToJointPosition":
                self.move_group.moveToJointPosition(self.joint_names, target_pose, wait=True)
            elif moveit_method == "moveToPose":
                self.end_effector_pose_stamped.pose = target_pose
                self.end_effector_pose_stamped.header.stamp = rospy.Time.now()
                self.move_group.moveToPose(self.end_effector_pose_stamped, self.end_effector_frame, wait=True)
            else:
                rospy.logerr("FetchController::control_arm_by_moveit : moveit_method not valid!")
                return False

            moveit_result = self.move_group.get_move_action().get_result()
            run_times += 1

            if not moveit_result:
                continue

            if moveit_result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Moving wrist Succeeded!")
                return True

            rospy.logerr("Arm goal in state: %s",
                         self.move_group.get_move_action().get_state())

        rospy.logerr("Try to Control Arm by MoveIt Failed!")
        return False

    def move_base_by_nav(self, target_pose):
        """ Navigation of fetch
        Args:
            target_pose: Pose
        Return:
            bool | success
        """
        self.move_base_goal.target_pose.pose = target_pose

        rospy.loginfo("Start Navigation...")
        for _ in range(self.max_waiting_times):
            self.move_base_client.send_goal(self.move_base_goal)
            finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(1))

            if finished_within_time:
                state = self.move_base_client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Nav Goal Succeeded!")
                    return True

        rospy.loginfo("Timed out achieving nav goal")
        self.move_base_client.cancel_all_goals()
        return False

    def directly_move(self, moving_speed, moving_dist):
        """ Let Fetch move directly
        Args:
            moving_speed: float
            moving_dist: float
        Return:
            bool | success
        """
        if moving_dist == 0:
            return True

        elif moving_speed > 0:
            linear_duration = moving_dist / moving_speed
        else:
            linear_duration = -moving_dist / moving_speed

        self.vel_cmd.linear.x = moving_speed

        sleep_rate = rospy.Rate(self.rate)

        ticks = int(linear_duration * self.rate)
        for _ in range(ticks):
            self.cmd_vel_pub.publish(self.vel_cmd)
            sleep_rate.sleep()
        return True

    def control_camera(self, camera_position, camera_direction, safe_dist=0, scanning_time=5, need_to_init=True):
        """ Control the camera with fetch
        Args:
            camera_position: list/tuple [x, y, z]
            camera_direction: list/tuple [x, y, z]
            wait_photo_time: float
            safe_dist: float
            need_to_init: bool
        Return:
            bool | success
        """
        base_target_pose = self.compute_base_target_pose(camera_position, camera_direction, safe_dist)
        end_effector_relative_pose = self.compute_end_effector_relative_pose(camera_position, camera_direction)

        if not self.control_arm_by_moveit("moveToJointPosition", self.shrink_arm_pose):
            rospy.logwarn("FetchController::control_camera : init arm failed!")
        if not self.move_base_by_nav(base_target_pose):
            rospy.logwarn("FetchController::control_camera : nav failed!")
        if not self.control_arm_by_moveit("moveToPose", end_effector_relative_pose):
            rospy.logwarn("FetchController::control_camera : move arm to shot image failed!")

        if not self.directly_move(self.directly_moving_speed, safe_dist):
            rospy.logwarn("FetchController::control_camera : directly move failed!")
        rospy.sleep(scanning_time)
        if not self.directly_move(-self.directly_moving_speed, safe_dist):
            rospy.logwarn("FetchController::control_camera : directly move back failed!")

        if need_to_init:
            if not self.control_arm_by_moveit("moveToJointPosition", self.shrink_arm_pose):
                rospy.logwarn("FetchController::control_camera : init arm failed!")

        self.move_group.get_move_action().cancel_all_goals()
        return True

    def moveToViewPoint(self, req):
        print("in moveToViewPoint!")
        camera_position = [req.view_point.position.x, req.view_point.position.y, req.view_point.position.z]
        camera_direction = [req.view_point.direction.x, req.view_point.direction.y, req.view_point.direction.z]
        self.control_camera(camera_position, camera_direction)
        print("moveToViewPoint process Finished!")
        return

    def startService(self):
        fetch_control_service = rospy.Service("fetch_" + self.robot_idx + "_control_service",
                                              ViewPointToBool,
                                              self.moveToViewPoint)
        rospy.spin()
        return True
    
    def test(self):
        self.control_camera([-1.5, 4, 1], [-2, 1, -0.5])
        rospy.sleep(1.0)
        self.control_camera([-2, 5, 1], [-2, -1, 0])
        return True

if __name__ == "__main__":
    rospy.init_node("multi_fetch_controller", anonymous=True)
    robot_idx = rospy.get_param("~robot_idx")
    fetch_controller = FetchController(robot_idx)
    fetch_controller.startService()

