//author: Arjun G Menon
//email:  agmenon@andrew.cmu.edu OR ajn.menon@gmail.com

#include "control/gripper_client.h"

Gripper::Gripper(){
  r_gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);
  while(!r_gripper_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
  }
  l_gripper_client_ = new GripperClient("l_gripper_controller/gripper_action", true);
  while(!l_gripper_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the l_gripper_controller/gripper_action action server to come up");
  }
}

Gripper::~Gripper(){
  delete r_gripper_client_;
  delete l_gripper_client_;
}

//Open the gripper
void Gripper::open(){
  pr2_controllers_msgs::Pr2GripperCommandGoal open;
  open.command.position = 0.08;
  open.command.max_effort = -1.0;  // Do not limit effort (negative)

  ROS_INFO("Sending open goal");
  r_gripper_client_->sendGoal(open);
  r_gripper_client_->waitForResult();
  if(r_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The right gripper opened!");
  else
    ROS_INFO("The right gripper failed to open.");

  l_gripper_client_->sendGoal(open);
  l_gripper_client_->waitForResult();
  if(l_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The left gripper opened!");
  else
    ROS_INFO("The left gripper failed to open.");
}

//Close the gripper
void Gripper::close(double position, double effort){
  pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
  squeeze.command.position = position;
  squeeze.command.max_effort = effort;

  ROS_INFO("Sending squeeze goal");
  r_gripper_client_->sendGoal(squeeze);
  r_gripper_client_->waitForResult();
  if(r_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The right gripper closed!");
  else
    ROS_INFO("The right gripper failed to close.");

  l_gripper_client_->sendGoal(squeeze);
  l_gripper_client_->waitForResult();
  if(l_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The left gripper closed!");
  else
    ROS_INFO("The left gripper failed to close.");
}

void Gripper::open_left(){
  pr2_controllers_msgs::Pr2GripperCommandGoal open;
  open.command.position = 0.08;
  open.command.max_effort = -1.0;  // Do not limit effort (negative)

  l_gripper_client_->sendGoal(open);
  l_gripper_client_->waitForResult();
  if(l_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The left gripper opened!");
  else
    ROS_INFO("The left gripper failed to open.");
}

void Gripper::close_left(double position, double effort){
  pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
  squeeze.command.position = position;
  squeeze.command.max_effort = effort;

  l_gripper_client_->sendGoal(squeeze);
  l_gripper_client_->waitForResult();
  if(l_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The left gripper closed!");
  else
    ROS_INFO("The left gripper failed to close.");
}

void Gripper::open_right(){
  pr2_controllers_msgs::Pr2GripperCommandGoal open;
  open.command.position = 0.08;
  open.command.max_effort = -1.0;  // Do not limit effort (negative)

  r_gripper_client_->sendGoal(open);
  r_gripper_client_->waitForResult();
  if(r_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The right gripper opened!");
  else
    ROS_INFO("The right gripper failed to open.");
}

//Close the gripper
void Gripper::close_right(double position, double effort){
  pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
  squeeze.command.position = position;
  squeeze.command.max_effort = effort;

  r_gripper_client_->sendGoal(squeeze);
  r_gripper_client_->waitForResult();
  if(r_gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The right gripper closed!");
  else
    ROS_INFO("The right gripper failed to close.");
}
