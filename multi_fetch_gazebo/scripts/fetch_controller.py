#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    scripts.fetch_controller
    ~~~~~~~~~~~~~~~~~~~~

    A class FetchController, implementing two methods to move fetch robot to the next viewpoint.

    :copyright: (c) 2018 by chLi.
    :license: MIT License, see LICENSE for more details.
"""

from math import sqrt

import rospy
import actionlib
import tf

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist #, PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelState, LinkState, LinkStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from std_srvs.srv import Empty
from gazebo_msgs.srv import SetPhysicsProperties, SetPhysicsPropertiesRequest, GetPhysicsProperties
# from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal

# ----gripper_frame----
# Swtich the frame to satisfy the pose given
# 'torso_lift_joint' or 'shoulder_pan_joint' or
# 'shoulder_lift_joint' or 'upperarm_roll_joint' or
# 'elbow_flex_joint' or 'forearm_roll_joint' or
# 'wrist_flex_joint' or 'wrist_roll_joint'
# 'wrist_roll_link' default

# ----group_models----
# Swtich the move_group to achieve the pose given
# 'arm' or 'arm_with_torso'
# 'arm_with_torso' default

# pylint: disable=line-too-long

robot_idx = input("Please set robot idx:")

class FetchController(object):

    """ Control the camera by controlling fetch """

    # init the fetch /// (str, str, str)
    def __init__(self, robot_idx, end_effector_frame='wrist_roll_link', group_name='arm_with_torso'):
        self.robot_idx = robot_idx
        self.received_data_odom = Odometry()
        self.received_data_jointstate = JointState()
        self.received_data_link_states = LinkStates()

        # parameters
        self.end_effector_frame = end_effector_frame
        self.group_name = group_name
        self.moving_mode = "GAZEBO_SET" # GAZEBO_SET/NAV_MOVEIT
        self.dist_between_base_and_camera = 0.5
        self.max_waiting_times = 180
        self.retry_times = 10

        self.current_arm_pose = None
        self.arm_pose_cache = {}
        self.arm_joint_trajectory_cache = {}
        self.arm_link_names = ('fetch::torso_lift_link', 'fetch::bellows_link', 'fetch::head_pan_link',
                               'fetch::head_tilt_link', 'fetch::shoulder_pan_link', 'fetch::shoulder_lift_link',
                               'fetch::upperarm_roll_link', 'fetch::elbow_flex_link', 'fetch::forearm_roll_link',
                               'fetch::wrist_flex_link', 'fetch::wrist_roll_link', 'fetch::l_gripper_finger_link',
                               'fetch::r_gripper_finger_link')

        self.model_state_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
        self.link_state_pub = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=30)
        self.trajectory_goal_pub = rospy.Publisher("/arm_with_torso_controller/follow_joint_trajectory/goal",
                                                   FollowJointTrajectoryActionGoal, queue_size=10)
        rospy.Subscriber("fetch_" + self.robot_idx + "/ground_truth", Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.link_states_callback, queue_size=1)

        self.move_base_client = None

        # wait for Publishers and Subscribers to construct
        rospy.sleep(1.0)

    def control_camera(self, camera_position, camera_direction, safe_dist=0, scanning_time=5, need_to_init=False):
        """ Control the camera with fetch
        Args:
            camera_position: list/tuple [x, y, z]
            camera_direction: list/tuple [x, y, z]
            wait_photo_time: float
            safe_dist: float
            need_to_init: bool
        """
        
        base_target_pose = self.compute_base_target_pose(camera_position, camera_direction, safe_dist)
        end_effector_relative_pose = self.compute_end_effector_relative_pose(camera_position, camera_direction)

        if self.moving_mode == "NAV_MOVEIT":
            directly_moving_speed = 0.5
            self.initialize_arm()

            self.change_base_pose_using_nav(base_target_pose)

        elif self.moving_mode == "GAZEBO_SET":
            safe_dist = 0
            self.change_base_pose_using_gazebo_set(base_target_pose)

        else:
            rospy.logerr("Invalid moving mode for fetch_" + self.robot_idx + "!")

        self.change_arm_pose_using_moveit(end_effector_relative_pose)

        if safe_dist != 0:
            self.directly_move(directly_moving_speed, safe_dist)

        rospy.sleep(scanning_time)

        if safe_dist != 0:
            self.directly_move(-directly_moving_speed, safe_dist)

        if need_to_init is True:
            self.initialize_fetch()

        # Create move group interface for a fetch robot.Two models there : arm && arm_with_torso
        move_group = MoveGroupInterface(
            self.group_name, "fetch_" + self.robot_idx + "/base_link",
            move_group="fetch_" + self.robot_idx + "/move_group")

        # This stops all arm movement goals
        move_group.get_move_action().cancel_all_goals()

    def change_base_pose_using_gazebo_set(self, target_pose):
        """ Change the base pose of Fetch
        Args:
            target_pose: Pose
        """

        # move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # rospy.loginfo("Waiting for move_base action server...")

        # wait_result = move_base_client.wait_for_server()

        # if not wait_result:
        #     rospy.loginfo("Can not connect to move base server")
        # else:
        #     rospy.loginfo("Connected to move base server")

        new_state = ModelState()
        new_state.model_name = 'fetch_' + self.robot_idx
        new_state.pose = target_pose
        new_state.twist.linear.x = 0
        new_state.twist.linear.y = 0
        new_state.twist.linear.z = 0
        new_state.twist.angular.x = 0
        new_state.twist.angular.y = 0
        new_state.twist.angular.z = 0
        new_state.reference_frame = "map"

        self.model_state_pub.publish(new_state)

        # self.received_data_odom.pose.pose.position.z = -10

        # while self.received_data_odom.pose.pose.position.z == -10:
        #     rospy.sleep(rospy.Duration(0.001))

        # odom = self.received_data_odom

        # odom.pose.pose.position.z = 0
        # odom.pose.pose.orientation.x = 0
        # odom.pose.pose.orientation.y = 0
        # odom.header.frame_id = 'map'
        # odom.child_frame_id = 'base_link'

        # initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

        # pose_with_cov_pub = PoseWithCovarianceStamped()

        # pose_with_cov_pub.pose = odom.pose
        # pose_with_cov_pub.header = odom.header

        # for _ in range(100):
        #     pose_with_cov_pub.header.stamp = rospy.Time().now()
        #     initialpose_pub.publish(pose_with_cov_pub)
        #     rospy.Rate(100).sleep()

        rospy.loginfo('Finish changing base pose')

    def change_arm_pose_using_gazebo_set(self, target_pose):
        """ Using /gazebo/set_link_states topic to change the fetch's arm pose
        Args:
            target_pose: Pose

        Only collected the result arm pose
        Only can send pose to /arm_with_torso_controller/follow_joint_trajectory/goal
        Can not change arm pose without time or process in gazebo
        """

        for pose, link_states in self.arm_pose_cache.items():
            if self.are_same_poses(pose, target_pose):
                self.pause_gazebo_physics2()
                for link_state in link_states:
                    self.link_state_pub.publish(link_state)
                    rospy.sleep(0.5)
                self.unpause_gazebo_physics2()
                rospy.loginfo('Finish changing arm pose')
                return

        self.change_arm_pose_using_moveit(target_pose)
        self.received_data_link_states.name = []
        while (not self.received_data_link_states.name) or (rospy.Time.now() - self.received_data_odom.header.stamp).to_sec() > 0.01:
            rospy.sleep(rospy.Duration(0.001))

        all_link_states = self.received_data_link_states
        base_pose = self.received_data_odom.pose.pose
        base_inverse_quat = tf.transformations.quaternion_inverse((base_pose.orientation.x, base_pose.orientation.y,
                                                                   base_pose.orientation.z, base_pose.orientation.w))
        arm_link_states = []

        for link_name, link_pose in zip(all_link_states.name, all_link_states.pose):
            if link_name in self.arm_link_names:
                arm_link_state = LinkState()
                arm_link_state.link_name = link_name
                arm_link_state.reference_frame = "fetch_" + self.robot_idx + "/base_link"
                relative_position = (link_pose.position.x - base_pose.position.x,
                                     link_pose.position.y - base_pose.position.y,
                                     link_pose.position.z - base_pose.position.z,
                                     0.0)

                relative_position = tf.transformations.quaternion_multiply(base_inverse_quat, relative_position)
                relative_position = tf.transformations.quaternion_multiply(relative_position,
                                                                           (base_pose.orientation.x,
                                                                            base_pose.orientation.y,
                                                                            base_pose.orientation.z,
                                                                            base_pose.orientation.w))

                arm_link_state.pose.position.x = relative_position[0]
                arm_link_state.pose.position.y = relative_position[1]
                arm_link_state.pose.position.z = relative_position[2]

                arm_link_quat = tf.transformations.quaternion_multiply(base_inverse_quat,
                                                                       (link_pose.orientation.x,
                                                                        link_pose.orientation.y,
                                                                        link_pose.orientation.z,
                                                                        link_pose.orientation.w))
                arm_link_state.pose.orientation.x = arm_link_quat[0]
                arm_link_state.pose.orientation.y = arm_link_quat[1]
                arm_link_state.pose.orientation.z = arm_link_quat[2]
                arm_link_state.pose.orientation.w = arm_link_quat[3]

                arm_link_state.twist.linear.x = 0.0
                arm_link_state.twist.linear.y = 0.0
                arm_link_state.twist.linear.z = 0.0
                arm_link_state.twist.angular.x = 0.0
                arm_link_state.twist.angular.y = 0.0
                arm_link_state.twist.angular.z = 0.0
                arm_link_states.append(arm_link_state)

        self.arm_pose_cache[target_pose] = arm_link_states


        # trajectory_goal_pub = rospy.Publisher('/arm_with_torso_controller/follow_joint_trajectory/goal',
        #                                       FollowJointTrajectoryActionGoal, queue_size=1)

        # # Construct a "pose_stamped" message as required by moveToPose
        # gripper_pose_stamped = PoseStamped()
        # gripper_pose_stamped.header.frame_id = 'base_link'

        # # Create move group interface for a fetch robot.Two models there : arm && arm_with_torso
        #  move_group = MoveGroupInterface(
        #      self.group_name, "fetch_" + self.robot_idx + "/base_link",
        #      move_group="fetch_" + self.robot_idx + "/move_group")

        # # Finish building the Pose_stamped message
        # gripper_pose_stamped.header.stamp = rospy.Time.now()

        # pose = Pose(Point(arm_goal_position[0], arm_goal_position[1], arm_goal_position[2]),
        #             self.compute_quaternion(arm_face_direction))

        # # Set the message pose
        # gripper_pose_stamped.pose = pose

        # rospy.loginfo("Starting moveit planning of wrist")

        # arm_result = JointTrajectoryPoint()

        # # Move gripper frame to the pose specified
        # move_group.moveToPose(gripper_pose_stamped, self.gripper_frame, plan_only=True)
        # moveit_result = move_group.get_move_action().get_result()

        # if moveit_result:
        #     if moveit_result.error_code.val == MoveItErrorCodes.SUCCESS:
        #         rospy.loginfo("Moveit Goal Succeeded!")
        #         arm_result = move_group.get_move_action().get_result().planned_trajectory.joint_trajectory.points[
        #             len(move_group.get_move_action().get_result().planned_trajectory.joint_trajectory.points) - 1]
        #     else:
        #         rospy.logerr("Arm goal in state: %s",
        #                      move_group.get_move_action().get_state())

        #         run_times = 0
        #         while moveit_result.error_code.val != MoveItErrorCodes.SUCCESS and run_times < retry_times:
        #             gripper_pose_stamped.header.stamp = rospy.Time.now()
        #             move_group.moveToPose(gripper_pose_stamped, self.gripper_frame)
        #             moveit_result = move_group.get_move_action().get_result()
        #             run_times = run_times + 1

        #             if moveit_result:
        #                 if moveit_result.error_code.val == MoveItErrorCodes.SUCCESS:
        #                     rospy.loginfo("Moveit Goal Succeeded!")
        #                     arm_result = move_group.get_move_action().get_result().planned_trajectory.joint_trajectory.points[
        #                         len(move_group.get_move_action().get_result().planned_trajectory.joint_trajectory.points) - 1]
        #                     break
        #                 else:
        #                     rospy.logerr("Arm goal in state: %s",
        #                                  move_group.get_move_action().get_state())
        # else:
        #     rospy.logerr("MoveIt! failure no result returned.")
        #     return

        # # jointstate_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)

        # rospy.Subscriber('/joint_states', JointState, self.jointstate_callback, queue_size=1)

        # while self.received_data_jointstate.header.seq == 0:
        #     rospy.sleep(rospy.Duration(0.001))

        # new_jointstate = self.received_data_jointstate

        # temp_torso_lift_joint = arm_result.positions[0]
        # temp_shoulder_pan_joint = arm_result.positions[1]
        # temp_shoulder_lift_joint = arm_result.positions[2]
        # temp_upperarm_roll_joint = arm_result.positions[3]
        # temp_elbow_flex_joint = arm_result.positions[4]
        # temp_forearm_roll_joint = arm_result.positions[5]
        # temp_wrist_flex_joint = arm_result.positions[6]
        # temp_wrist_roll_joint = arm_result.positions[7]

        # new_jointstate.position = [new_jointstate.position[0], new_jointstate.position[1],
        #                            temp_torso_lift_joint, new_jointstate.position[3],
        #                            new_jointstate.position[4], new_jointstate.position[5],
        #                            temp_shoulder_pan_joint, temp_shoulder_lift_joint,
        #                            temp_upperarm_roll_joint, temp_elbow_flex_joint,
        #                            temp_forearm_roll_joint, temp_wrist_flex_joint,
        #                            temp_wrist_roll_joint, new_jointstate.position[13],
        #                            new_jointstate.position[14]]

        # new_trajectory_goal = FollowJointTrajectoryActionGoal()

        # new_trajectory_goal.header.frame_id = 'base_link'
        # new_trajectory_goal.header.stamp = rospy.Time.now()
        # new_trajectory_goal.goal.trajectory.header = new_trajectory_goal.header
        # new_trajectory_goal.goal.trajectory.points.append(arm_result)
        # new_trajectory_goal.goal.trajectory.joint_names = ["torso_lift_joint", "shoulder_pan_joint",
        #                                              "shoulder_lift_joint", "upperarm_roll_joint",
        #                                              "elbow_flex_joint", "forearm_roll_joint",
        #                                              "wrist_flex_joint", "wrist_roll_joint"]

        # trajectory_goal_pub.publish(new_trajectory_goal)

        # for _ in range(100):
        #     new_jointstate.header.stamp = rospy.Time().now()
        #     # jointstate_pub.publish(new_jointstate)
        #     rospy.Rate(100).sleep()

        # rospy.loginfo('Arm changed finished')

    def change_base_pose_using_nav(self, target_pose):
        """ Navigation of fetch
        Args:
            target_pose: Pose
        """
        
        if not self.move_base_client:
            print("start connect movebase client")
            self.move_base_client = actionlib.SimpleActionClient(
                "fetch_" + self.robot_idx + "/move_base", MoveBaseAction)
            rospy.loginfo("Waiting for move_base action server...")
            wait_result = self.move_base_client.wait_for_server()
            if wait_result:
                rospy.loginfo("Connected to move base server")
            else:
                rospy.loginfo("Can not connect to move base server")
                self.move_base_client = None

        if self.move_base_client:
            move_base_goal = MoveBaseGoal()
            move_base_goal.target_pose.header.frame_id = 'map'
            move_base_goal.target_pose.pose = target_pose

            rospy.loginfo("Starting Navigation")

            for _ in range(self.max_waiting_times):
                self.move_base_client.send_goal(move_base_goal)
                finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(1))

                if finished_within_time:
                    break

            if not finished_within_time:
                rospy.loginfo("Timed out achieving nav goal")
                self.move_base_client.cancel_all_goals()
            else:
                state = self.move_base_client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Nav Goal Succeeded!")

    def change_arm_pose_using_moveit(self, target_pose):
        """ Using Moveit to change fetch's arm pose
        Args:
            target_pose: Pose
        """
        
        if self.current_arm_pose is not None and self.are_same_poses(self.current_arm_pose, target_pose):
            return

        # Using follow joint trajectory to save time when target pose is in cache
        # for poses, cached_trajectory in self.arm_joint_trajectory_cache.items():
        #     if self.are_same_poses(poses[0], self.current_arm_pose) and self.are_same_poses(poses[1], target_pose):
        #         new_trajectory_goal = FollowJointTrajectoryActionGoal()
        #         new_trajectory_goal.header.frame_id = "base_link"
        #         new_trajectory_goal.header.stamp = rospy.Time.now()
        #         new_trajectory_goal.goal.trajectory = cached_trajectory

        #         self.trajectory_goal_pub.publish(new_trajectory_goal)
        #         self.current_arm_pose = target_pose
        #         rospy.loginfo('Finish changing arm pose')
        #         return

        # Otherwise, using original Moveit
        # Define ground plane
        planning_scene = PlanningSceneInterface(
            "base_link",
            "fetch_" + self.robot_idx)
        planning_scene.removeCollisionObject("my_front_ground")
        planning_scene.removeCollisionObject("my_back_ground")
        planning_scene.removeCollisionObject("my_right_ground")
        planning_scene.removeCollisionObject("my_left_ground")
        planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
        planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
        planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
        planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

        # Construct a "pose_stamped" message as required by moveToPose
        end_effector_pose_stamped = PoseStamped()
        end_effector_pose_stamped.header.frame_id = 'fetch_' + self.robot_idx + '/base_link'

        # Create move group interface for a fetch robot.Two models there : arm && arm_with_torso
        move_group = MoveGroupInterface(
            self.group_name, "fetch_" + self.robot_idx + "/base_link",
            move_group="fetch_" + self.robot_idx + "/move_group")

        # Finish building the Pose_stamped message
        end_effector_pose_stamped.header.stamp = rospy.Time.now()
        # Set the message pose
        end_effector_pose_stamped.pose = target_pose

        rospy.loginfo("Starting moveit planning of wrist")

        # Move end effector frame to the pose specified
        move_group.moveToPose(end_effector_pose_stamped, self.end_effector_frame)
        moveit_result = move_group.get_move_action().get_result()

        if moveit_result:
            if moveit_result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Moveit Goal Succeeded!")
                # self.arm_joint_trajectory_cache[target_pose] = \
                #         self.accelerate_arm_trajectory(move_group.get_move_action().get_result().planned_trajectory.joint_trajectory)
                # self.arm_joint_trajectory_cache[(self.current_arm_pose, target_pose)] = \
                #         move_group.get_move_action().get_result().planned_trajectory.joint_trajectory
                self.current_arm_pose = target_pose
            else:
                rospy.logerr("Arm goal in state: %s",
                             move_group.get_move_action().get_state())

                run_times = 0
                while moveit_result.error_code.val != MoveItErrorCodes.SUCCESS and run_times < self.retry_times:
                    end_effector_pose_stamped.header.stamp = rospy.Time.now()
                    move_group.moveToPose(end_effector_pose_stamped, self.end_effector_frame)
                    moveit_result = move_group.get_move_action().get_result()
                    run_times = run_times + 1

                    if moveit_result:
                        if moveit_result.error_code.val == MoveItErrorCodes.SUCCESS:
                            rospy.loginfo("Moveit Goal Succeeded!")
                            # self.arm_joint_trajectory_cache[target_pose] = \
                            #         self.accelerate_arm_trajectory(move_group.get_move_action().get_result().planned_trajectory.joint_trajectory)
                            # self.arm_joint_trajectory_cache[(self.current_arm_pose, target_pose)] = \
                            #         move_group.get_move_action().get_result().planned_trajectory.joint_trajectory
                            self.current_arm_pose = target_pose
                            break
                        else:
                            rospy.logerr("Arm goal in state: %s",
                                         move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt! failure no result returned.")

    def directly_move(self, moving_speed, moving_dist):
        """ Let Fetch move directly
        Args:
            moving_speed: float
            moving_dist: float
        """

        cmd_vel_pub = rospy.Publisher('fetch_' + self.robot_idx + '/cmd_vel', Twist, queue_size=1)

        # Hz
        rate = 50
        sleep_rate = rospy.Rate(rate)

        if moving_speed == 0:
            moving_speed = 1
            linear_duration = 0
        elif moving_speed > 0:
            linear_duration = moving_dist / moving_speed
        else:
            linear_duration = -moving_dist / moving_speed

        vel_cmd = Twist()

        vel_cmd.linear.x = moving_speed
        vel_cmd.linear.y = 0
        vel_cmd.linear.z = 0

        vel_cmd.angular.x = 0
        vel_cmd.angular.y = 0
        vel_cmd.angular.z = 0

        ticks = int(linear_duration * rate)

        for _ in range(ticks):
            cmd_vel_pub.publish(vel_cmd)
            sleep_rate.sleep()

    def initialize_arm(self):
        """ Initialize the arm of the fetch """

        # Support the init pose of fetch
        init_moveit_pose = [0, 2, 1.6, 0, 1.57, 0, 1.8, 0] # shrink arm
        #  init_moveit_pose = [0, -0.595508, -0.914319, 1.15083, 1.52896, 0.688713, -0.660242, -1.23424] # extend arm

        # TF joint names
        joint_names = ["torso_lift_joint", "shoulder_pan_joint",
                       "shoulder_lift_joint", "upperarm_roll_joint",
                       "elbow_flex_joint", "forearm_roll_joint",
                       "wrist_flex_joint", "wrist_roll_joint"]

        # Create move group interface for a fetch robot.Two models there : arm && arm_with_torso
        move_group = MoveGroupInterface(
            "arm_with_torso", "fetch_" + self.robot_idx + "/base_link",
            move_group="fetch_" + self.robot_idx + "/move_group")

        if rospy.is_shutdown():
            return

        rospy.loginfo("Starting initialization of wrist")

        # Plans the joints in joint_names to angles in pose
        move_group.moveToJointPosition(joint_names, init_moveit_pose, wait=True)

        moveit_result = move_group.get_move_action().get_result()

        if moveit_result:
            if moveit_result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Wrist initialization Succeeded!")
            else:
                rospy.logerr("Arm goal in state: %s",
                             move_group.get_move_action().get_state())

                run_times = 0
                while moveit_result.error_code.val != MoveItErrorCodes.SUCCESS and run_times < self.retry_times:
                    if rospy.is_shutdown():
                        break

                    move_group.moveToJointPosition(joint_names, init_moveit_pose, wait=True)

                    moveit_result = move_group.get_move_action().get_result()
                    run_times = run_times + 1

                    if moveit_result:
                        if moveit_result.error_code.val == MoveItErrorCodes.SUCCESS:
                            rospy.loginfo("Wrist initialization Succeeded!")
                            break
                        else:
                            rospy.logerr("Arm goal in state: %s",
                                         move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt! failure no result returned.")

    def initialize_fetch(self):
        """ Initialize fetch """

        init_pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))

        self.initialize_arm()

        if self.moving_mode == "GAZEBO_SET":
            self.change_base_pose_using_gazebo_set(init_pose)
        elif self.moving_mode == "NAV_MOVEIT":
            self.change_base_pose_using_nav(init_pose)
        else:
            rospy.logerr("Invalid moving mode for fetch!")

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
                          camera_position[1] - (self.dist_between_base_and_camera + safe_dist) * xy_plane_vector[1], 0),
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

        xy = sqrt(face_to_direction[0] * face_to_direction[0] + face_to_direction[1] * face_to_direction[1])
        xyz = sqrt(face_to_direction[0] * face_to_direction[0] + face_to_direction[1] * face_to_direction[1] + face_to_direction[2] * face_to_direction[2])
        if xyz == 0:
            return Quaternion(0, 0, 0, 1)
        if xy == 0:
            if face_to_direction[2] > 0:
                return Quaternion(0, -1 / sqrt(2), 0, 1 / sqrt(2))
            return Quaternion(0, 1 / sqrt(2), 0, 1 / sqrt(2))
        else:
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

    def are_same_poses(self, pose1, pose2):
        """ Judge whether two poses are equal
        Args:
            pose1: Pose
            pose2: Pose
        Return:
            bool, True if two poses are equal
        """
        tolerance = 0.01
        if abs(pose1.position.x - pose2.position.x) > tolerance:
            return False
        if abs(pose1.position.y - pose2.position.y) > tolerance:
            return False
        if abs(pose1.position.z - pose2.position.z) > tolerance:
            return False
        if abs(pose1.orientation.x - pose2.orientation.x) > tolerance:
            return False
        if abs(pose1.orientation.y - pose2.orientation.y) > tolerance:
            return False
        if abs(pose1.orientation.z - pose2.orientation.z) > tolerance:
            return False
        if abs(pose1.orientation.w - pose2.orientation.w) > tolerance:
            return False
        return True

    # def run_before_close_gravity(self):
    #     """ Change fetch to the special pose to take photos
    #         """

    #     pose = Pose(Point(0.5, 0, 0.5), self.compute_quaternion([1, 0, 0]))

    #     self.moveit_of_fetch(pose, self.gripper_frame, self.group_models, 10)

    # def run_after_open_gravity(self):
    #     """ Change fetch to the correct pose from the sky
    #         """

    #     self.change_pose_to([0, 0, 0], [1, 0, 0])

    # def move_camera_without_process(self, camera_pose, camera_look_at, wait_photo_time):
    #     """ Control the camera without process
    #             Args:
    #                 camera_pose: list [x, y, z]
    #                 camera_look_at: list [x, y, z]]
    #                 wait_photo_time: float
    #             """

    #     length_look_at = sqrt(camera_look_at[0]*camera_look_at[0] + camera_look_at[1]*camera_look_at[1]
    #                           + camera_look_at[2]*camera_look_at[2])
    #     if length_look_at == 0:
    #         length_look_at = 1

    #     unit_look_at = [camera_look_at[0] / length_look_at, camera_look_at[1] / length_look_at,
    #                     camera_look_at[2] / length_look_at]

    #     if unit_look_at[0] != 0 or unit_look_at[1] != 0:
    #         vertical_look_at = [unit_look_at[0]*unit_look_at[2], unit_look_at[1]*unit_look_at[2],
    #                             unit_look_at[2]*unit_look_at[2] - 1]

    #     length_vertical = sqrt(vertical_look_at[0]*vertical_look_at[0] +
    #                             vertical_look_at[1]*vertical_look_at[1] +
    #                             vertical_look_at[2]*vertical_look_at[2])
    #     if vertical_look_at == 0:
    #         vertical_look_at = 1

    #     unit_vertical_look_at = [vertical_look_at[0] / length_vertical, vertical_look_at[1] / length_vertical,
    #                                 vertical_look_at[2] / length_vertical]

    #     fetch_pose = [camera_pose[0] - 0.5 * unit_look_at[0] + 0.5 * unit_vertical_look_at[0],
    #                     camera_pose[1] - 0.5 * unit_look_at[1] + 0.5 * unit_vertical_look_at[1],
    #                     camera_pose[2] - 0.5 * unit_look_at[2] + 0.5 * unit_vertical_look_at[2]]

    #     self.change_pose_to(fetch_pose, camera_look_at)

    #     rospy.sleep(wait_photo_time)

    ### callback functions ###
    def odom_callback(self, data):
        """ Receive and save the data subscribed
        Args:
            data: Nothing needed to input
        """

        self.received_data_odom = data

    def link_states_callback(self, data):
        """ Receive and save the data subscribed
        Args:
            data: Nothing needed to input
        """

        self.received_data_link_states = data

    def jointstate_callback(self, data):
        """ Receive and save the data subscribed
        Args:
            data: Nothing needed to input
        """

        self.received_data_jointstate = data

    def pause_gazebo_physics1(self):
        rospy.wait_for_service("/gazebo/pause_physics")
        pause_physics = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        try:
            pause_physics()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def unpause_gazebo_physics1(self):
        rospy.wait_for_service("/gazebo/unpause_physics")
        unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        try:
            unpause_physics()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def pause_gazebo_physics2(self):
        rospy.wait_for_service("/gazebo/get_physics_properties")
        get_physics = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)
        old_physics_properties = get_physics()
        new_physics_properties = SetPhysicsPropertiesRequest()
        new_physics_properties.time_step = old_physics_properties.time_step
        new_physics_properties.max_update_rate = old_physics_properties.max_update_rate
        new_physics_properties.gravity = old_physics_properties.gravity
        new_physics_properties.gravity.z = 0.0
        new_physics_properties.ode_config = old_physics_properties.ode_config
        new_physics_properties.ode_config.sor_pgs_iters = 0

        rospy.wait_for_service("/gazebo/set_physics_properties")
        set_physics = rospy.ServiceProxy("/gazebo/set_physics_properties", SetPhysicsProperties)
        try:
            set_physics(new_physics_properties)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def unpause_gazebo_physics2(self):
        rospy.wait_for_service("/gazebo/get_physics_properties")
        get_physics = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)
        old_physics_properties = get_physics()
        new_physics_properties = SetPhysicsPropertiesRequest()
        new_physics_properties.time_step = old_physics_properties.time_step
        new_physics_properties.max_update_rate = old_physics_properties.max_update_rate
        new_physics_properties.gravity = old_physics_properties.gravity
        new_physics_properties.gravity.z = -9.8
        new_physics_properties.ode_config = old_physics_properties.ode_config
        new_physics_properties.ode_config.sor_pgs_iters = 50

        rospy.wait_for_service("/gazebo/set_physics_properties")
        set_physics = rospy.ServiceProxy("/gazebo/set_physics_properties", SetPhysicsProperties)
        try:
            set_physics(new_physics_properties)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def accelerate_arm_trajectory(self, trajectory):
        acceleration_ratio = 1.0
        new_trajectory = trajectory
        for trajectory_point in new_trajectory.points:
            new_velocities = []
            new_accelerations = []
            for vel in trajectory_point.velocities:
                new_velocities.append(vel * acceleration_ratio)
            for acc in trajectory_point.accelerations:
                new_accelerations.append(acc * acceleration_ratio)
            trajectory_point.velocities = tuple(new_velocities)
            trajectory_point.accelerations = tuple(new_accelerations)
            trajectory_point.time_from_start /= acceleration_ratio
        return new_trajectory
                

    # @property
    # def moving_mode(self):
    #     return self.moving_mode

    # @moving_mode.setter
    # def moving_mode(self, value):
    #     if value == "GAZEBO_SET" or value == "NAV_MOVEIT":
    #         self.moving_mode = value
    #     else:
    #         rospy.logerr("Invalid moving mode for fetch!")

def test():
    """ Test function for module testing

        Run 'run_before_close_gravity' only
        Go to gazebo to close the gravity
        Then run 'move_camera_without_process' for several times
        Then go to gazebo to open the gravity(fetch will fly, ignore this temporarily,I'll fix it later)
        Then run 'run_after_open_gravity'
    """

    rospy.init_node("fetch_" + robot_idx + "_test", anonymous=True)

    
    fetch_ctrl = FetchController(robot_idx)
    # fetch_ctrl.initialize_fetch()
    #  fetch_ctrl.moving_mode = "GAZEBO_SET"
    fetch_ctrl.moving_mode = "NAV_MOVEIT"
    fetch_ctrl.control_camera([-1.5, 4, 1], [-2, 1, -0.5], 0)
    # rospy.sleep(1.0)
    fetch_ctrl.control_camera([-2, 5, 1], [-2, -1, 0], 0)
    # rospy.sleep(1.0)
    # fetch_ctrl.control_camera([-2, 3, 1], [-2, 1, -0.5], 0)
    # rospy.sleep(1.0)
    # fetch_ctrl.control_camera([-3, 3, 1], [-2, 1, -0.5], 0)
    # fetch_ctrl.initialize_fetch()
    # rospy.sleep(1.0)
    # fetch_ctrl.control_camera([1, 1, 1], [-2, 1, 0.5], 0)
    #fetch_ctrl.change_pose_to([1,2,0], [1,1,0])

    # fetch_ctrl.run_before_close_gravity()

    # fetch_ctrl.move_camera_without_process([3, 2, 2], [1, 1, 1], 5)
    # fetch_ctrl.move_camera_without_process([1, 1, 1], [-1, 1, 5], 5)

    # fetch_ctrl.run_after_open_gravity()

    # fetch_ctrl.camera_ctrl_with_fetch([3.5, 2, 1.2], [1, 1, 0], 5, 0, True)


if __name__ == "__main__":
    test()
