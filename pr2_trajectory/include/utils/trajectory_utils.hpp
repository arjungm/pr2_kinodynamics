//author: Arjun G Menon
//email:  agmenon@andrew.cmu.edu OR ajn.menon@gmail.com
#ifndef TRAJ_UTILS_HPP
#define TRAJ_UTILS_HPP

#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

namespace trajutils{

  //initializes a goal object with the right arm control stuff
  void addJointNamesRightArm(pr2_controllers_msgs::JointTrajectoryGoal &goal, int size);
  void addJointNamesLeftArm(pr2_controllers_msgs::JointTrajectoryGoal &goal, int size);
    
  //takes a vector-string of joint values and inputs them into a waypoint
  void inputRightArmJointValues(pr2_controllers_msgs::JointTrajectoryGoal &goal, int index, std::vector<std::string> &tokens);
  
  void inputRightArmJointValuesSorted(pr2_controllers_msgs::JointTrajectoryGoal &goal, int index, std::vector<double> &tokens);
  void inputLeftArmJointValuesSorted(pr2_controllers_msgs::JointTrajectoryGoal &goal, int index, std::vector<double> &tokens);
  
  //zeros the positions of a waypoint
  void zeroRightArmPositions(pr2_controllers_msgs::JointTrajectoryGoal &goal, int index);

  //zeros the velocities of a given waypoint
  void zeroRightArmVelocities(pr2_controllers_msgs::JointTrajectoryGoal &goal, int index);
    
  //copies the waypoint at copy_index to index , only positions and velocities
  void copyRightArmWaypoint(pr2_controllers_msgs::JointTrajectoryGoal &goal, int index, int copy_index, bool copy_velocity);

  void copyWaypointToWaypoint(const pr2_controllers_msgs::JointTrajectoryGoal &traj1, pr2_controllers_msgs::JointTrajectoryGoal &traj2, int index1, int index2);

  void scaleVelocities(pr2_controllers_msgs::JointTrajectoryGoal &traj, double scale, int index);
  
  void printTrajectory(std::string name, pr2_controllers_msgs::JointTrajectoryGoal &goal);
}//namespace
#endif
