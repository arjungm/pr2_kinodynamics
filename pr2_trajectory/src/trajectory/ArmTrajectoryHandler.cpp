//author: Arjun G Menon
//email:  agmenon@andrew.cmu.edu OR ajn.menon@gmail.com
#include "trajectory/ArmTrajectoryHandler.hpp"
#include "utils/trajectory_utils.hpp"
#include "utils/utils.hpp"
#include <stdlib.h>

static std::string JOINT_TRAJECTORY_ACTION = "_arm_controller/joint_trajectory_action";

ArmTrajectoryHandler::ArmTrajectoryHandler(std::string write_path): node_handle_("~")
{
  // tell the action client that we want to spin a thread by default
  right_arm_traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
  left_arm_traj_client_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);

  // wait for action server to come up
  while(!right_arm_traj_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the joint_trajectory_action server");
  }
  while(!left_arm_traj_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the joint_trajectory_action server");
  }

  setLoggingLocation(write_path);
  
  execution_start_time_ = ros::Time::now().toSec();
  is_recording_ =false;
  is_right_arm_client_active_ = false;
}

void ArmTrajectoryHandler::setLoggingLocation(std::string write_path){
  //add timestamp to the logs manually
  time_t clock;
  time(&clock);
  std::string time(ctime(&clock));
  time.erase(time.size()-1, 1);
  write_path = write_path + "/" + time;

  //create the trajectory streaming files
  std::stringstream pos_file_name;  pos_file_name << write_path << "_pos.txt";
  std::stringstream vel_file_name;  vel_file_name << write_path << "_vel.txt";
  std::stringstream eff_file_name;  eff_file_name << write_path << "_eff.txt";

  //open the ofstreams
  position_log.open(    pos_file_name.str().c_str()   );
  velocity_log.open(    vel_file_name.str().c_str()   );
  effort_log.open(      eff_file_name.str().c_str()   );
}

void ArmTrajectoryHandler::closeLoggingFiles(){
  position_log.close();
  velocity_log.close();
  effort_log.close();
}

//Clean up the action client
ArmTrajectoryHandler::~ArmTrajectoryHandler(){
  delete right_arm_traj_client_;
  delete left_arm_traj_client_;
}

//Sends the command to start a given trajectory
void ArmTrajectoryHandler::startTrajectory(JointTrajectory traj)
{
  // When to start the trajectory: 0.5s from now
  traj.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);
  //check which trajectory client needs to send the command to
  if(is_right_arm_client_active_)
    right_arm_traj_client_->sendGoal(traj);
  else
    left_arm_traj_client_->sendGoal(traj);
  //enable logging and record start time for controller execution
  is_recording_ = true;
  execution_start_time_ = traj.trajectory.header.stamp.toSec();
}

void ArmTrajectoryHandler::acceptTrajectoryFromPlanner(std::vector<std::vector<double> > &planner_traj){
  std::cout << planner_traj.size() << " waypoints in plan" << std::endl;
  JointTrajectory traj;
  //check which arm we are accepting the trajectory
  if(is_right_arm_client_active_)
    trajutils::addJointNamesRightArm(traj, planner_traj.size());
  else
    trajutils::addJointNamesLeftArm(traj, planner_traj.size());

  //copy
  for(size_t waypoint_index=0; waypoint_index < planner_traj.size(); waypoint_index++){
    trajutils::inputRightArmJointValuesSorted(traj, waypoint_index, planner_traj.at( waypoint_index ));
    traj.trajectory.points[waypoint_index].time_from_start = ros::Duration(planner_traj.at( waypoint_index ).at(14)); //XXX hard coded
  }

  //push into the list of trajectories
  trajectory_history_.push_back(traj);
}

void ArmTrajectoryHandler::readTrajectoryFromFile(std::ifstream &input_stream){
  //turn plaintext trajectory into JointTrajectory
  std::vector<std::vector<double> > traj;
  std::vector<std::string> tokens;
  for(std::string line; std::getline(input_stream, line); )
  {
    string_utils::split(line, ' ', tokens);
    std::vector<double> waypoint(tokens.size());
    for(size_t i=0; i<tokens.size(); i++)
    {
      waypoint.at(i) = atof(tokens.at(i).c_str());
    }
    traj.push_back(waypoint);
  }
  acceptTrajectoryFromPlanner(traj);
}

void ArmTrajectoryHandler::setLeftActive(){ is_right_arm_client_active_ = false; }

void ArmTrajectoryHandler::setRightActive(){ is_right_arm_client_active_ = true; }

ArmTrajectoryHandler::JointTrajectory ArmTrajectoryHandler::getLastTrajectory(){
  return trajectory_history_.at(trajectory_history_.size()-1); 
}

ArmTrajectoryHandler::JointTrajectory ArmTrajectoryHandler::getLastTrajectoryReversed(){
  JointTrajectory traj_reverse;
  JointTrajectory traj_forward(trajectory_history_.at(trajectory_history_.size()-1));

  //fill in the positions and zero the velocities
  if(is_right_arm_client_active_)
    trajutils::addJointNamesLeftArm(traj_reverse, traj_forward.trajectory.points.size());
  else
    trajutils::addJointNamesRightArm(traj_reverse, traj_forward.trajectory.points.size());
  
  //iterate backwards
  int N = traj_reverse.trajectory.points.size() -1 ;
  for(size_t i = 0; i< traj_reverse.trajectory.points.size(); i+=1)
  {
      trajutils::copyWaypointToWaypoint(traj_forward, traj_reverse, N-i, i);
      trajutils::scaleVelocities(traj_reverse, 0.0, i);
  }

  //compute timing information in reverse
  std::vector<double> timings;
  for(int i=0; i<N+1; i++)
    timings.push_back(traj_forward.trajectory.points[i].time_from_start.toSec());

  double total_time = timings.at(N);

  for(int i=0; i<N+1; i++)
    timings.at(i) = -(timings.at(i) - total_time);
  
  std::reverse(timings.begin(), timings.end());
  
  //fill in the timing
  for(size_t i=0; i<traj_reverse.trajectory.points.size(); i++){
    traj_reverse.trajectory.points[i].time_from_start = ros::Duration(timings.at(i));
  }
  
  return traj_reverse;
}

ArmTrajectoryHandler::JointTrajectory ArmTrajectoryHandler::getStartingTrajectory()
{ 
  //TODO throw a ros warn, this shouldn't be executed except in simulation
  JointTrajectory initial_trajectory;
  JointTrajectory last_trajectory(trajectory_history_.at(trajectory_history_.size()-1));

  if(is_right_arm_client_active_)
    trajutils::addJointNamesRightArm(initial_trajectory , 1);
  else
    trajutils::addJointNamesLeftArm(initial_trajectory, 1);

  trajutils::copyWaypointToWaypoint(last_trajectory, initial_trajectory, 0,0);
  trajutils::scaleVelocities( initial_trajectory, 0.0, 0);
  initial_trajectory.trajectory.points[0].time_from_start = ros::Duration(4.0); //conservatively set a long time

  return initial_trajectory;
}

//! Returns the current state of the action
actionlib::SimpleClientGoalState ArmTrajectoryHandler::getState()
{
  if(is_right_arm_client_active_)
    return right_arm_traj_client_->getState();
  else
    return left_arm_traj_client_->getState();
}

void ArmTrajectoryHandler::JointStateCallback(const sensor_msgs::JointState &js_msg){
  double time = ros::Time::now().toSec() - execution_start_time_;
  //positions 
  if(is_recording_){
    position_log << time << " ";
    for(std::vector<double>::const_iterator pit = js_msg.position.begin(); pit!=js_msg.position.end(); ++pit){
      position_log << *pit << " ";
    }
    position_log << "\n";

    //write velocities
    velocity_log << time << " ";
    for(std::vector<double>::const_iterator vit = js_msg.velocity.begin(); vit!=js_msg.velocity.end(); ++vit){
      velocity_log << *vit << " ";
    }
    velocity_log << "\n";

    //write accelerations
    effort_log << time << " ";
    for(std::vector<double>::const_iterator ait = js_msg.effort.begin(); ait!=js_msg.effort.end(); ++ait){
      effort_log << *ait << " ";
    }
    effort_log << "\n";
  }
}

void ArmTrajectoryHandler::print_array(const KDL::JntArray &q)
{
  for(unsigned int i=0; i<q.rows(); i++)
    std::cout << q.data(i,0) << " ";
  std::cout << std::endl;
}
