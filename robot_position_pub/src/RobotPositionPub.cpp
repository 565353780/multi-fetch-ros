#include "robot_position_pub/RobotPositionPub.h"

bool RobotPositionPub::setRobotParam(
    const std::string &world_name,
    const std::string &robot_name,
    const size_t &robot_num,
    const std::string& robot_position_topic_name,
    const std::string& robot_position_ground_truth_topic_name,
    const size_t& need_odom)
{
  world_name_ = world_name;
  robot_name_ = robot_name;
  robot_num_ = robot_num;
  robot_position_topic_name_ = robot_position_topic_name;
  robot_position_ground_truth_topic_name_ = robot_position_ground_truth_topic_name;
  need_odom_ = need_odom;

  last_pub_tf_time_vec_.resize(robot_num_, ros::Time());

  return true;
}

bool RobotPositionPub::startSync()
{
  //0 : TFOnly
  //0 : ApproximateTime
  //1 : TimeSynchronizer
  size_t sync_mode = 0;

  switch(sync_mode)
  {
    case 0:
    {
      std::vector<ros::Subscriber> robot_ground_turth_sub_vec;

      for(size_t robot_idx = 0; robot_idx < robot_num_; ++robot_idx)
      {
        ros::Subscriber camera_ground_truth_sub;
        robot_ground_turth_sub_vec.emplace_back(camera_ground_truth_sub);

        robot_ground_turth_sub_vec[robot_idx] = nh.subscribe<Odometry>(
            robot_name_ + std::to_string(robot_idx) + "/" + robot_position_ground_truth_topic_name_,
            1,
            [this, robot_idx](const auto& msg){ this->tfOnlyCallback(msg, robot_idx); });
      }

      ros::spin();

      break;
    }
    case 1:
    {
      typedef sync_policies::ApproximateTime<Odometry, Odometry> MySyncPolicy;
      std::vector<std::unique_ptr<message_filters::Subscriber<Odometry>>> odom_sub_vec;
      std::vector<std::unique_ptr<message_filters::Subscriber<Odometry>>> robot_ground_truth_sub_vec;

      std::vector<std::unique_ptr<Synchronizer<MySyncPolicy>>> sync_vec;

      for(size_t robot_idx = 0; robot_idx < robot_num_; ++robot_idx)
      {
        if(need_odom_ == 1)
        {
          odom_sub_vec.emplace_back(new message_filters::Subscriber<Odometry>(
                nh, robot_name_ + std::to_string(robot_idx) + "/odom", sub_queue_size_));
        }
        else
        {
          odom_sub_vec.emplace_back(new message_filters::Subscriber<Odometry>(
                nh, world_name_, sub_queue_size_));
        }

        robot_ground_truth_sub_vec.emplace_back(new message_filters::Subscriber<Odometry>(
              nh, robot_name_ + std::to_string(robot_idx) + "/" + robot_position_ground_truth_topic_name_, sub_queue_size_));

        sync_vec.emplace_back(new Synchronizer<MySyncPolicy>(MySyncPolicy(sub_queue_size_),
                                                             *odom_sub_vec[robot_idx],
                                                             *robot_ground_truth_sub_vec[robot_idx]));
        
        sync_vec[robot_idx]->registerCallback(boost::bind(
              &RobotPositionPub::unionCallback, this, _1, _2, robot_idx));
      }

      ros::spin();

      break;
    }
    case 2:
    {
      typedef TimeSynchronizer<Odometry, Odometry> MySyncPolicy;

      std::vector<std::unique_ptr<message_filters::Subscriber<Odometry>>> odom_sub_vec;
      std::vector<std::unique_ptr<message_filters::Subscriber<Odometry>>> robot_ground_truth_sub_vec;

      std::vector<std::unique_ptr<MySyncPolicy>> sync_vec;

      for(size_t robot_idx = 0; robot_idx < robot_num_; ++robot_idx)
      {
        if(need_odom_ == 1)
        {
          odom_sub_vec.emplace_back(new message_filters::Subscriber<Odometry>(
                nh, robot_name_ + std::to_string(robot_idx) + "/odom", sub_queue_size_));
        }
        else
        {
          odom_sub_vec.emplace_back(new message_filters::Subscriber<Odometry>(
                nh, world_name_, sub_queue_size_));
        }

        robot_ground_truth_sub_vec.emplace_back(new message_filters::Subscriber<Odometry>(
              nh, robot_name_ + std::to_string(robot_idx) + "/" + robot_position_ground_truth_topic_name_, sub_queue_size_));

        sync_vec.emplace_back(new MySyncPolicy(*odom_sub_vec[robot_idx],
                                               *robot_ground_truth_sub_vec[robot_idx],
                                               sub_queue_size_));
        
        sync_vec[robot_idx]->registerCallback(boost::bind(
              &RobotPositionPub::unionCallback, this, _1, _2, robot_idx));
      }

      ros::spin();

      break;
    }
  }

  ros::spin();

  return true;
}

void RobotPositionPub::tfOnlyCallback(
    const OdometryConstPtr& robot_ground_truth,
    size_t robot_idx)
{
  const float& robot_position_x = robot_ground_truth->pose.pose.position.x;
  const float& robot_position_y = robot_ground_truth->pose.pose.position.y;
  const float& robot_position_z = robot_ground_truth->pose.pose.position.z;

  tf2::Quaternion q_map_to_baselink;
  tf2::convert(robot_ground_truth->pose.pose.orientation, q_map_to_baselink);

  geometry_msgs::TransformStamped transformStamped_map_to_odom;
  transformStamped_map_to_odom.header.frame_id = world_name_;
  transformStamped_map_to_odom.child_frame_id = robot_name_ + std::to_string(robot_idx) + "/odom";
  transformStamped_map_to_odom.transform.translation.x = 0;
  transformStamped_map_to_odom.transform.translation.y = 0;
  transformStamped_map_to_odom.transform.translation.z = 0;
  transformStamped_map_to_odom.transform.rotation.x = 0;
  transformStamped_map_to_odom.transform.rotation.y = 0;
  transformStamped_map_to_odom.transform.rotation.z = 0;
  transformStamped_map_to_odom.transform.rotation.w = 1;
  transformStamped_map_to_odom.header.stamp = robot_ground_truth->header.stamp;

  geometry_msgs::TransformStamped transformStamped_odom_to_baselink;

  if(need_odom_ == 1)
  {
    transformStamped_odom_to_baselink.header.frame_id = robot_name_ + std::to_string(robot_idx) + "/odom";
  }
  else
  {
    transformStamped_odom_to_baselink.header.frame_id = world_name_;
  }

  transformStamped_odom_to_baselink.child_frame_id = robot_name_ + std::to_string(robot_idx) + "/" + robot_position_topic_name_;
  transformStamped_odom_to_baselink.transform.translation.x = robot_position_x;
  transformStamped_odom_to_baselink.transform.translation.y = robot_position_y;
  transformStamped_odom_to_baselink.transform.translation.z = robot_position_z;
  transformStamped_odom_to_baselink.transform.rotation.x = q_map_to_baselink.x();
  transformStamped_odom_to_baselink.transform.rotation.y = q_map_to_baselink.y();
  transformStamped_odom_to_baselink.transform.rotation.z = q_map_to_baselink.z();
  transformStamped_odom_to_baselink.transform.rotation.w = q_map_to_baselink.w();
  transformStamped_odom_to_baselink.header.stamp = robot_ground_truth->header.stamp;
  if(transformStamped_odom_to_baselink.header.stamp == last_pub_tf_time_vec_[robot_idx])
  {
    return;
  }
  if(need_odom_ == 1)
  {
    tf_pub_.sendTransform(transformStamped_map_to_odom);
  }
  tf_pub_.sendTransform(transformStamped_odom_to_baselink);
  last_pub_tf_time_vec_[robot_idx] = transformStamped_odom_to_baselink.header.stamp;

  // tf2::Quaternion q_baselink_to_camera;
  // q_baselink_to_camera.setEuler(PI / 2.0, 0, -PI / 2.0);

  // geometry_msgs::TransformStamped transformStamped_baselink_to_camera;
  // transformStamped_baselink_to_camera.header.frame_id = robot_name_ + std::to_string(robot_idx);
  // transformStamped_baselink_to_camera.child_frame_id = robot_name_ + "frame_" + std::to_string(robot_idx);
  // transformStamped_baselink_to_camera.transform.translation.x = 0;
  // transformStamped_baselink_to_camera.transform.translation.y = 0;
  // transformStamped_baselink_to_camera.transform.translation.z = 0;
  // transformStamped_baselink_to_camera.transform.rotation.x = q_baselink_to_camera.x();
  // transformStamped_baselink_to_camera.transform.rotation.y = q_baselink_to_camera.y();
  // transformStamped_baselink_to_camera.transform.rotation.z = q_baselink_to_camera.z();
  // transformStamped_baselink_to_camera.transform.rotation.w = q_baselink_to_camera.w();
  // transformStamped_baselink_to_camera.header.stamp = robot_ground_truth->header.stamp;
  // tf_pub_.sendTransform(transformStamped_baselink_to_camera);

  // const std::string output_msg = "tfOnlyCallback : I heard srobot" + std::to_string(robot_idx) + "'s sync msgs.";
  // ROS_INFO(output_msg.c_str());
}

void RobotPositionPub::unionCallback(
    const OdometryConstPtr& odom_ground_truth,
    const OdometryConstPtr& robot_ground_truth,
    size_t robot_idx)
{
  const float& robot_position_x = robot_ground_truth->pose.pose.position.x;
  const float& robot_position_y = robot_ground_truth->pose.pose.position.y;
  const float& robot_position_z = robot_ground_truth->pose.pose.position.z;

  const float& odom_position_x = odom_ground_truth->pose.pose.position.x;
  const float& odom_position_y = odom_ground_truth->pose.pose.position.y;
  const float& odom_position_z = odom_ground_truth->pose.pose.position.z;

  tf2::Quaternion q_odom = tf2::Quaternion(
      odom_ground_truth->pose.pose.orientation.x,
      odom_ground_truth->pose.pose.orientation.y,
      odom_ground_truth->pose.pose.orientation.z,
      odom_ground_truth->pose.pose.orientation.w);

  tf2::Quaternion q_map_to_robot = tf2::Quaternion(
      robot_ground_truth->pose.pose.orientation.x,
      robot_ground_truth->pose.pose.orientation.y,
      robot_ground_truth->pose.pose.orientation.z,
      robot_ground_truth->pose.pose.orientation.w);

  tf2::Quaternion q_map_to_odom = q_map_to_robot * q_odom.inverse();

  geometry_msgs::TransformStamped transformStamped_map_to_odom;
  transformStamped_map_to_odom.header.frame_id = world_name_;
  transformStamped_map_to_odom.child_frame_id = robot_name_ + std::to_string(robot_idx) + "/odom";
  transformStamped_map_to_odom.transform.translation.x = robot_position_x - odom_position_x;
  transformStamped_map_to_odom.transform.translation.y = robot_position_y - odom_position_y;
  transformStamped_map_to_odom.transform.translation.z = robot_position_z - odom_position_z;
  transformStamped_map_to_odom.transform.rotation.x = q_map_to_odom.x();
  transformStamped_map_to_odom.transform.rotation.y = q_map_to_odom.y();
  transformStamped_map_to_odom.transform.rotation.z = q_map_to_odom.z();
  transformStamped_map_to_odom.transform.rotation.w = q_map_to_odom.w();
  transformStamped_map_to_odom.header.stamp = robot_ground_truth->header.stamp;
  if(transformStamped_map_to_odom.header.stamp == last_pub_tf_time_vec_[robot_idx])
  {
    return;
  }
  tf_pub_.sendTransform(transformStamped_map_to_odom);
  last_pub_tf_time_vec_[robot_idx] = transformStamped_map_to_odom.header.stamp;

  // tf2::Quaternion q_baselink_to_camera;
  // q_baselink_to_camera.setEuler(PI / 2.0, 0, -PI / 2.0);

  // geometry_msgs::TransformStamped transformStamped_baselink_to_camera;
  // transformStamped_baselink_to_camera.header.frame_id = robot_name_ + std::to_string(robot_idx);
  // transformStamped_baselink_to_camera.child_frame_id = robot_name_ + "frame_" + std::to_string(robot_idx);
  // transformStamped_baselink_to_camera.transform.translation.x = 0;
  // transformStamped_baselink_to_camera.transform.translation.y = 0;
  // transformStamped_baselink_to_camera.transform.translation.z = 0;
  // transformStamped_baselink_to_camera.transform.rotation.x = q_baselink_to_camera.x();
  // transformStamped_baselink_to_camera.transform.rotation.y = q_baselink_to_camera.y();
  // transformStamped_baselink_to_camera.transform.rotation.z = q_baselink_to_camera.z();
  // transformStamped_baselink_to_camera.transform.rotation.w = q_baselink_to_camera.w();
  // transformStamped_baselink_to_camera.header.stamp = current_time;
  // tf_pub_.sendTransform(transformStamped_baselink_to_camera);

  // const std::string output_msg = "unionCallback : I heard srobot" + std::to_string(robot_idx) + "'s sync msgs.";
  // ROS_INFO(output_msg.c_str());
}

