//author: Arjun G Menon
//email:  agmenon@andrew.cmu.edu OR ajn.menon@gmail.com
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include "trajectory/ArmTrajectoryHandler.hpp"
#include "control/gripper_client.h"
#include "control/torso_client.h"
#include <yaml-cpp/yaml.h>

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh_;

  TorsoController mTorso; //moves the torso ...
  Gripper gripper;
  mTorso.up(); //false means it will move up til we kill the ros::ok loop
  gripper.open();

  bool run_point = true;

  if(argc<3)
  {
    run_point = false;
  }
  else if(argc<2)
  {
    std::cout << "Usage: run_trajectory <yaml file>" << std::endl;
    return 0;
  }

  std::string path_to_record;
  std::string path_to_plan;

  std::ifstream input_yaml(argv[1]);
  YAML::Parser parser_yaml(input_yaml);
  YAML::Node doc;
  if(parser_yaml.GetNextDocument(doc))
  {
    doc["path_to_record"] >> path_to_record;
    doc["path_to_plan"] >> path_to_plan;

  }
  else
  {
    std::cout << "Invalid yaml file passed in!" << std::endl;
  }

  ArmTrajectoryHandler arm_traj_handler(path_to_record);
  arm_traj_handler.setRightActive();
  
  // read the trajectory
  std::ifstream input_trajectory_file(path_to_plan.c_str());
  arm_traj_handler.readTrajectoryFromFile(input_trajectory_file);

  //send to start
  arm_traj_handler.startTrajectory(arm_traj_handler.getStartingTrajectory());
  while(!arm_traj_handler.getState().isDone() && ros::ok())
  {
    usleep(5000);
  }
  std::cout << "Ready to execute...";
  if(!run_point)
    std::cin.get();

  //execute
  ros::Subscriber sub = nh_.subscribe("/joint_states", 1, &ArmTrajectoryHandler::JointStateCallback, &arm_traj_handler);
  arm_traj_handler.startTrajectory(arm_traj_handler.getLastTrajectory());
  double start_time = ros::Time::now().toSec();
  bool command_not_sent = true;
  while(!arm_traj_handler.getState().isDone() && ros::ok())
  {
    double elapsed_time = ros::Time::now().toSec()-start_time;
    if(elapsed_time > 4.1 && command_not_sent)
    {
      if(!run_point)
        gripper.close(0.06,-1);
      command_not_sent = false;
    }
    ros::spinOnce(); //spin for jointstate callback
    usleep(5000);
  }
  std::cout << "Ready to conclude...";

  //send to start
  arm_traj_handler.startTrajectory(arm_traj_handler.getLastTrajectoryReversed());
  while(!arm_traj_handler.getState().isDone() && ros::ok())
  {
    usleep(5000);
  }

  //conclude
  arm_traj_handler.closeLoggingFiles();
}
