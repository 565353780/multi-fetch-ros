#ifndef ROBOT_POSITION_PUB_H
#define ROBOT_POSITION_PUB_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <string>
#include <vector>

#define PI 3.14159265358979

using namespace message_filters;
using namespace nav_msgs;

class RobotPositionPub
{
public:
  RobotPositionPub()
  {
    sub_queue_size_ = 10;
    pub_rate_ = 1000;
  }

  bool setRobotParam(
      const std::string& world_name,
      const std::string& robot_name,
      const size_t& robot_num,
      const std::string& robot_position_topic_name,
      const size_t& need_odom);

  bool startSync();

private:
  void tfOnlyCallback(
      const OdometryConstPtr& robot_ground_truth,
      size_t robot_idx);

  void unionCallback(
      const OdometryConstPtr& odom_ground_truth,
      const OdometryConstPtr& robot_ground_truth,
      size_t robot_idx);

public:
  ros::NodeHandle nh;

private:
  size_t sub_queue_size_;
  size_t pub_rate_;

  tf2_ros::TransformBroadcaster tf_pub_;

  std::string world_name_;
  size_t robot_num_;
  std::string robot_name_;
  std::string robot_position_topic_name_;
  size_t need_odom_;

  std::vector<ros::Time> last_pub_tf_time_vec_;
};

#endif // ROBOT_POSITION_PUB_H
