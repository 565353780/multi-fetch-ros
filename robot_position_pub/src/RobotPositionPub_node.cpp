#include "robot_position_pub/RobotPositionPub.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_position_pub");

  RobotPositionPub robot_position_pub;

  std::string world_name = "";
  std::string robot_name = "";
  size_t robot_num = 0;

  if(argc > 1)
  {
    world_name = argv[1];
  }

  if(argc > 2)
  {
    robot_name = argv[2];
  }

  if(argc > 3)
  {
    robot_num = atoi(argv[3]);
  }

  if(world_name == "" || robot_name == "" || robot_num == 0)
  {
    std::cout << "RobotPositionPub::main :\n" <<
      "input not valid!\n";

    return -1;
  }

  robot_position_pub.setRobotParam(world_name, robot_name, robot_num);

  robot_position_pub.startSync();

  return 0;
}

