//author: Arjun G Menon
//email:  agmenon@andrew.cmu.edu OR ajn.menon@gmail.com

#include "utils/trajectory_utils.hpp"
  
//initializes a goal object with the right arm control stuff
void trajutils::addJointNamesRightArm(pr2_controllers_msgs::JointTrajectoryGoal &goal, int size){
  goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
  goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
  goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
  goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
  goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
  goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
  goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
  goal.trajectory.points.resize(size);
}
//initializes a goal object with the right arm control stuff
void trajutils::addJointNamesLeftArm(pr2_controllers_msgs::JointTrajectoryGoal &goal, int size){
  goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
  goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
  goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
  goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
  goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
  goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
  goal.trajectory.joint_names.push_back("l_wrist_roll_joint");
  goal.trajectory.points.resize(size);
}

//takes a vector-string of joint values and inputs them into a waypoint
void trajutils::inputRightArmJointValues(pr2_controllers_msgs::JointTrajectoryGoal &goal, int index, std::vector<std::string> &tokens){
  //TODO make it agnostic to the size of tokens
  goal.trajectory.points[index].positions.resize(7);
  goal.trajectory.points[index].positions[0] = atof(tokens.at(17).c_str());
  goal.trajectory.points[index].positions[1] = atof(tokens.at(18).c_str());
  goal.trajectory.points[index].positions[2] = atof(tokens.at(16).c_str());
  goal.trajectory.points[index].positions[3] = atof(tokens.at(20).c_str());
  goal.trajectory.points[index].positions[4] = atof(tokens.at(19).c_str());
  goal.trajectory.points[index].positions[5] = atof(tokens.at(21).c_str());
  goal.trajectory.points[index].positions[6] = atof(tokens.at(22).c_str());
}

//TODO
void trajutils::inputRightArmJointValuesSorted(pr2_controllers_msgs::JointTrajectoryGoal &goal, int index, std::vector<double> &tokens){
   goal.trajectory.points[index].positions.resize(7);
   goal.trajectory.points[index].positions[0] = tokens.at(0);
   goal.trajectory.points[index].positions[1] = tokens.at(1);
   goal.trajectory.points[index].positions[2] = tokens.at(2);
   goal.trajectory.points[index].positions[3] = tokens.at(3);
   goal.trajectory.points[index].positions[4] = tokens.at(4);
   goal.trajectory.points[index].positions[5] = tokens.at(5);
   goal.trajectory.points[index].positions[6] = tokens.at(6);

   if(tokens.size()>7){
     goal.trajectory.points[index].velocities.resize(7);
     goal.trajectory.points[index].velocities[0] = tokens.at(7);
     goal.trajectory.points[index].velocities[1] = tokens.at(8);
     goal.trajectory.points[index].velocities[2] = tokens.at(9);
     goal.trajectory.points[index].velocities[3] = tokens.at(10);
     goal.trajectory.points[index].velocities[4] = tokens.at(11);
     goal.trajectory.points[index].velocities[5] = tokens.at(12);
     goal.trajectory.points[index].velocities[6] = tokens.at(13);
   }
}//TODO
void trajutils::inputLeftArmJointValuesSorted(pr2_controllers_msgs::JointTrajectoryGoal &goal, int index, std::vector<double> &tokens){
   goal.trajectory.points[index].positions.resize(7);
   goal.trajectory.points[index].positions[0] = tokens.at(0);
   goal.trajectory.points[index].positions[1] = tokens.at(1);
   goal.trajectory.points[index].positions[2] = tokens.at(2);
   goal.trajectory.points[index].positions[3] = tokens.at(3);
   goal.trajectory.points[index].positions[4] = tokens.at(4);
   goal.trajectory.points[index].positions[5] = tokens.at(5);
   goal.trajectory.points[index].positions[6] = tokens.at(6);

   if(tokens.size()>7){
     goal.trajectory.points[index].velocities.resize(7);
     goal.trajectory.points[index].velocities[0] = tokens.at(7);
     goal.trajectory.points[index].velocities[1] = tokens.at(8);
     goal.trajectory.points[index].velocities[2] = tokens.at(9);
     goal.trajectory.points[index].velocities[3] = tokens.at(10);
     goal.trajectory.points[index].velocities[4] = tokens.at(11);
     goal.trajectory.points[index].velocities[5] = tokens.at(12);
     goal.trajectory.points[index].velocities[6] = tokens.at(13);
   }
}


void trajutils::copyWaypointToWaypoint(const pr2_controllers_msgs::JointTrajectoryGoal &traj1, pr2_controllers_msgs::JointTrajectoryGoal &traj2, int index1, int index2){
  
   traj2.trajectory.points[index2].positions.resize(7);
   traj2.trajectory.points[index2].positions[0] = traj1.trajectory.points[index1].positions[0]; 
   traj2.trajectory.points[index2].positions[1] = traj1.trajectory.points[index1].positions[1];
   traj2.trajectory.points[index2].positions[2] = traj1.trajectory.points[index1].positions[2];
   traj2.trajectory.points[index2].positions[3] = traj1.trajectory.points[index1].positions[3];
   traj2.trajectory.points[index2].positions[4] = traj1.trajectory.points[index1].positions[4];
   traj2.trajectory.points[index2].positions[5] = traj1.trajectory.points[index1].positions[5];
   traj2.trajectory.points[index2].positions[6] = traj1.trajectory.points[index1].positions[6];

   traj2.trajectory.points[index2].velocities.resize(7);
   traj2.trajectory.points[index2].velocities[0] = traj1.trajectory.points[index1].velocities[0];
   traj2.trajectory.points[index2].velocities[1] = traj1.trajectory.points[index1].velocities[1];
   traj2.trajectory.points[index2].velocities[2] = traj1.trajectory.points[index1].velocities[2];
   traj2.trajectory.points[index2].velocities[3] = traj1.trajectory.points[index1].velocities[3];
   traj2.trajectory.points[index2].velocities[4] = traj1.trajectory.points[index1].velocities[4];
   traj2.trajectory.points[index2].velocities[5] = traj1.trajectory.points[index1].velocities[5];
   traj2.trajectory.points[index2].velocities[6] = traj1.trajectory.points[index1].velocities[6];
}

void trajutils::scaleVelocities(pr2_controllers_msgs::JointTrajectoryGoal &traj, double scale, int index){
   traj.trajectory.points[index].velocities[0] = scale*traj.trajectory.points[index].velocities[0];
   traj.trajectory.points[index].velocities[1] = scale*traj.trajectory.points[index].velocities[1];
   traj.trajectory.points[index].velocities[2] = scale*traj.trajectory.points[index].velocities[2];
   traj.trajectory.points[index].velocities[3] = scale*traj.trajectory.points[index].velocities[3];
   traj.trajectory.points[index].velocities[4] = scale*traj.trajectory.points[index].velocities[4];
   traj.trajectory.points[index].velocities[5] = scale*traj.trajectory.points[index].velocities[5];
   traj.trajectory.points[index].velocities[6] = scale*traj.trajectory.points[index].velocities[6];
}

//zeros the positions of a given waypoint
void trajutils::zeroRightArmPositions(pr2_controllers_msgs::JointTrajectoryGoal &goal, int index){
  goal.trajectory.points[index].positions.resize(7);
  for (size_t j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].positions[j] = 0.0;
  }
}


//zeros the velocities of a given waypoint
void trajutils::zeroRightArmVelocities(pr2_controllers_msgs::JointTrajectoryGoal &goal, int index){
  goal.trajectory.points[index].velocities.resize(7);
  for (size_t j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
}

//copies the waypoint at copy_index to index , only positions and velocities
void trajutils::copyRightArmWaypoint(pr2_controllers_msgs::JointTrajectoryGoal &goal, int index, int copy_index, bool copy_velocity){
  goal.trajectory.points[index].positions.resize(7);
  if(copy_velocity)
    goal.trajectory.points[index].velocities.resize(7);
  for(int ind = 0; ind < 7; ind++){
    goal.trajectory.points[index].positions[ind] = goal.trajectory.points[copy_index].positions[ind];
    if(copy_velocity)
      goal.trajectory.points[index].velocities[ind] = goal.trajectory.points[copy_index].velocities[ind];
  }
}

void trajutils::printTrajectory(std::string name, pr2_controllers_msgs::JointTrajectoryGoal &goal)
{
  std::cout << name << ":" << std::endl;
  for(size_t i=0; i < goal.trajectory.points.size(); i++)
  {
    std::cout << "[";
    for(size_t j=0; j < 6; j++)
      std::cout << goal.trajectory.points[i].positions[j] << " ";
    std::cout << goal.trajectory.points[i].positions[6] << "] [";
    for(size_t j=0; j < 6; j++)
      std::cout << goal.trajectory.points[i].velocities[j] << " ";
    std::cout << goal.trajectory.points[i].velocities[6] << "] [";
    std::cout << goal.trajectory.points[i].time_from_start.toSec() << "]" << std::endl;
  }
}

