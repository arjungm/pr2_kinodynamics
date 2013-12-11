//author: Arjun G Menon
//email:  agmenon@andrew.cmu.edu OR ajn.menon@gmail.com
#ifndef EXEC_TRAJ_HPP
#define EXEC_TRAJ_HPP
#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <fstream>

//dynamics
#include <kdl/jntarray.hpp>

class ArmTrajectoryHandler{

  typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;
  typedef pr2_controllers_msgs::JointTrajectoryGoal JointTrajectory;
  typedef std::vector<std::vector<double> > RawJointTrajectory;
  
  private:
    bool is_recording_;
    bool is_right_arm_client_active_;
    double execution_start_time_;

    //streamed recording of position, velocity, and effort data of the PR2 arm during trajectory execution
    std::ofstream position_log;
    std::ofstream velocity_log;
    std::ofstream effort_log;

    //trajectory clients for the left and right arms
    TrajClient* right_arm_traj_client_;
    TrajClient* left_arm_traj_client_;

    //trajectory representations
    JointTrajectory current_trajectory_;
    RawJointTrajectory current_raw_trajectory_;

    //history of provided trajectory goals
    std::vector<JointTrajectory> trajectory_history_;

    //ros node handle
    ros::NodeHandle node_handle_;
  
  public:
    ArmTrajectoryHandler(std::string write_path);
    ~ArmTrajectoryHandler();

    //display
    void print_array(const KDL::JntArray &q);

    //trajectory execution
    void startTrajectory(JointTrajectory traj);
    void setRightActive();
    void setLeftActive();

    //trajectory parsers from arm planner
    void acceptTrajectoryFromPlanner(std::vector<std::vector<double> > &traj);
    void readTrajectoryFromFile(std::ifstream &input_stream);

    //trajectory modifiers
    JointTrajectory getLastTrajectory();
    JointTrajectory getLastTrajectoryReversed();
    JointTrajectory getStartingTrajectory();
    
    //controller info
    actionlib::SimpleClientGoalState getState();
    
    //trajectory logging 
    void setLoggingLocation(std::string write_path);
    void closeLoggingFiles();
    void JointStateCallback(const sensor_msgs::JointState &js_msg); //for monitoring execute progress
};
#endif

