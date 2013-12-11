//author: Arjun G Menon
//email:  agmenon@andrew.cmu.edu OR ajn.menon@gmail.com
#ifndef DYN_TRAH_HPP
#define DYN_TRAJ_HPP
#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <fstream>

//dynamics
#include <pr2_mechanism_model/chain.h>
#include <pr2_mechanism_model/robot.h>
#include <kdl/chain.hpp>

#include "dynamics_model.h"

#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>

class RobotArm{

  typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;
  
  private:
    TrajClient* traj_client_;
  
  public:
    RobotArm(std::string &robot_description_string, double planning_timestep);
    ~RobotArm();
    //display
    void print_array(const KDL::JntArray &q);
    //traj executors
    void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal);
    //traj creators
    pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory();
    pr2_controllers_msgs::JointTrajectoryGoal armResetTrajectory();
    actionlib::SimpleClientGoalState getState();
    //traj recorders
    void close_files();
    void JointStateCallback(const sensor_msgs::JointState &js_msg);
    std::ofstream pos_data_file;
    std::ofstream vel_data_file;
    std::ofstream acc_data_file;
    std::ofstream true_file;
    bool record_start_;
    double exp_start_time_;
    //arm param ptrs
    boost::shared_ptr<KDL::Vector> grav_ptr;
    boost::shared_ptr<KDL::Chain> kdl_chain_ptr;
    DynamicsModel::Ptr dyn_model_ptr;
    //arm params
    pr2_mechanism_model::Chain chain_;
    KDL::JntArray q_;
    KDL::JntArray qdot_;
    KDL::JntArray qdotdot_;
};
#endif

