//author: Arjun G Menon
//email:  agmenon@andrew.cmu.edu OR ajn.menon@gmail.com
#include "dynamic_trajectory.hpp"

//dynamics
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <Eigen/Geometry>


RobotArm::RobotArm(std::string &robot_description_string, double exp_start_time)
{
  // tell the action client that we want to spin a thread by default
  traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

  // wait for action server to come up
  while(!traj_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the joint_trajectory_action server");
  }

  //filenames
  std::stringstream pos_file_name;
  pos_file_name << "data/Dynamic/pos.txt";
  std::stringstream vel_file_name;
  vel_file_name << "data/Dynamic/vel.txt";
  std::stringstream acc_file_name;
  acc_file_name << "data/Dynamic/acc.txt";
  std::stringstream true_file_name;
  true_file_name << "data/Dynamic/true.txt";

  pos_data_file.open(pos_file_name.str().c_str());
  vel_data_file.open(vel_file_name.str().c_str());
  acc_data_file.open(acc_file_name.str().c_str());
  true_file.open(true_file_name.str().c_str());

  //kinematic chain initialization
  std::string root_name = "torso_lift_link";
  //std::string root_name = "r_forearm_link";
  std::string tip_name = "r_wrist_roll_link";

  //parse urdf 
  std::cout << "\tParsing urdf." << std::endl;
  KDL::Tree modeltree;
  if (!kdl_parser::treeFromString(robot_description_string, modeltree))
    ROS_ERROR("Failed to construct kdl tree");

  //create kdl_chain_
  kdl_chain_ptr = boost::make_shared<KDL::Chain>();
  std::cout << "\tComposing chain." << std::endl;
  if (!modeltree.getChain(root_name,tip_name,*kdl_chain_ptr))
    ROS_ERROR("could not find a chain from '%s' to '%s'",root_name.c_str(), tip_name.c_str());

//  std::cout << "Segments: ";
//  for(unsigned int i = 0; i< kdl_chain_ptr->getNrOfSegments(); i++)
//    std::cout << kdl_chain_ptr->segments.at(i).getName() << " ";
//  std::cout << std::endl;
  
  std::cout << "Joints: ";
  for(unsigned int i = 0; i< kdl_chain_ptr->getNrOfSegments(); i++)
    std::cout << kdl_chain_ptr->segments.at(i).getJoint().getName() << " ";
  std::cout << std::endl;
  
  grav_ptr = boost::make_shared<KDL::Vector>(0.0,0.0,-9.81);
  std::cout << "\tCreating dynamics model." << std::endl;
  dyn_model_ptr= boost::make_shared<DynamicsModel>(kdl_chain_ptr, grav_ptr);

  exp_start_time_ = exp_start_time;
  record_start_ = false;
}


void RobotArm::close_files(){
  pos_data_file.close();
  vel_data_file.close();
  acc_data_file.close();
}

//! Clean up the action client
RobotArm::~RobotArm()
{
  delete traj_client_;
}

//! Sends the command to start a given trajectory
void RobotArm::startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
{
  // When to start the trajectory: 1s from now
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  record_start_=true;
  traj_client_->sendGoal(goal);
}

pr2_controllers_msgs::JointTrajectoryGoal RobotArm::armExtensionTrajectory()
{
  //our goal variable
  pr2_controllers_msgs::JointTrajectoryGoal goal;
  // First, the joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
  goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
  goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
  goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
  goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
  goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
  goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
  //generate the trajectory
  std::cout << "Initializing robot arm states for dynamics..." << std::endl;
  q_.resize(kdl_chain_ptr->getNrOfJoints());
  qdot_.resize(kdl_chain_ptr->getNrOfJoints());
  qdotdot_.resize(kdl_chain_ptr->getNrOfJoints());
  //initial q and qdot are zeros
  KDL::SetToZero(q_);
  KDL::SetToZero(qdot_);
  //Select input joint torques
  std::cout << "Setting input joint torques..." << std::endl;
  
  KDL::JntArray G_(kdl_chain_ptr->getNrOfJoints());
  dyn_model_ptr->cdp_ptr->JntToGravity(q_, G_);

  //forward simulate
  double time_applied_fwd = 1;
  double time_applied_bwd = 1;
  double time_step = 0.1;
  size_t num_points_fwd = time_applied_fwd/time_step;
  size_t num_points_bwd = time_applied_bwd/time_step;

  goal.trajectory.points.resize(num_points_fwd + num_points_bwd);
  KDL::JntArray qp(q_);
  KDL::JntArray qpdot(qdot_);
  KDL::JntArray qn(kdl_chain_ptr->getNrOfJoints());
  KDL::JntArray qndot(kdl_chain_ptr->getNrOfJoints());

  std::cout << "Forward Simulating..." << std::endl;
  dyn_model_ptr->set_delta(time_step);

  //forward
  double t = 0.0;
  for(size_t ind = 0; ind < goal.trajectory.points.size(); ind++)
  {
    //init
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].velocities.resize(7);
    //set acceleration inputs
    KDL::SetToZero(dyn_model_ptr->qdotdot);
    if(ind > num_points_fwd)
      dyn_model_ptr->qdotdot.data(3,0)=0.5;
    else
      dyn_model_ptr->qdotdot.data(3,0)=-1;

    //integrate
    dyn_model_ptr->integrate(qp, qpdot, qn, qndot);
    //copy
    goal.trajectory.points[ind].positions[0]=qn.data(0,0);
    goal.trajectory.points[ind].positions[1]=qn.data(1,0);
    goal.trajectory.points[ind].positions[2]=qn.data(2,0);
    goal.trajectory.points[ind].positions[3]=qn.data(3,0);
    goal.trajectory.points[ind].positions[4]=qn.data(4,0);
    goal.trajectory.points[ind].positions[5]=qn.data(5,0);
    goal.trajectory.points[ind].positions[6]=qn.data(6,0);
    goal.trajectory.points[ind].velocities[0]=qndot.data(0,0);
    goal.trajectory.points[ind].velocities[1]=qndot.data(1,0);
    goal.trajectory.points[ind].velocities[2]=qndot.data(2,0);
    goal.trajectory.points[ind].velocities[3]=qndot.data(3,0);
    goal.trajectory.points[ind].velocities[4]=qndot.data(4,0);
    goal.trajectory.points[ind].velocities[5]=qndot.data(5,0);
    goal.trajectory.points[ind].velocities[6]=qndot.data(6,0);
    qp=qn;
    qpdot=qndot;
    //timing
    t = t + time_step;
    goal.trajectory.points[ind].time_from_start = ros::Duration(t);
  }

  for(size_t i = 0; i < goal.trajectory.points.size(); i++){
    true_file << goal.trajectory.points[i].positions[3] << " ";
    true_file << goal.trajectory.points[i].velocities[3] << " ";
    true_file << goal.trajectory.points[i].time_from_start << "\n";
  }

  //we are done; return the goal
  return goal;
}

pr2_controllers_msgs::JointTrajectoryGoal RobotArm::armResetTrajectory()
{
  //our goal variable
  pr2_controllers_msgs::JointTrajectoryGoal goal;

  // First, the joint names, which apply to all waypoints
  goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
  goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
  goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
  goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
  goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
  goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
  goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

  goal.trajectory.points.resize(1);

  //start
  goal.trajectory.points[0].positions.resize(7);
  for(size_t ind2 = 0; ind2 < 7; ind2++)
  {
    goal.trajectory.points[0].positions[ind2] = 0.0;
  }
  goal.trajectory.points[0].time_from_start = ros::Duration(5.0);

  //we are done; return the goal
  return goal;
}

//! Returns the current state of the action
actionlib::SimpleClientGoalState RobotArm::getState()
{
  return traj_client_->getState();
}

void RobotArm::JointStateCallback(const sensor_msgs::JointState &js_msg){
  double time = ros::Time::now().toSec() - exp_start_time_;
  //positions 
  if(record_start_){
    pos_data_file << time << " ";
    for(std::vector<double>::const_iterator pit = js_msg.position.begin(); pit!=js_msg.position.end(); ++pit){
      pos_data_file << *pit << " ";
    }
    pos_data_file << "\n";

    //write velocities
    vel_data_file << time << " ";
    for(std::vector<double>::const_iterator vit = js_msg.velocity.begin(); vit!=js_msg.velocity.end(); ++vit){
      vel_data_file << *vit << " ";
    }
    vel_data_file << "\n";

    //write accelerations
    acc_data_file << time << " ";
    for(std::vector<double>::const_iterator ait = js_msg.effort.begin(); ait!=js_msg.effort.end(); ++ait){
      acc_data_file << *ait << " ";
    }
    acc_data_file << "\n";
  }
}

void RobotArm::print_array(const KDL::JntArray &q)
{
  for(unsigned int i=0; i<q.rows(); i++)
    std::cout << q.data(i,0) << " ";
  std::cout << std::endl;
}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");
  
  // Start the trajectory
  ros::NodeHandle n;
  double exp_time = ros::Time::now().toSec();

  //get robot state
  std::cout << "Getting the urdf..." << std::endl;
  std::string urdf_param_name="robot_description";
  std::string urdf_string;
  n.getParam(urdf_param_name,urdf_string);

  //create the arm planner class
  std::cout << "Creating Robot Arm Trajectory Planner..." << std::endl;
  RobotArm arm(urdf_string, exp_time);
  ros::Subscriber sub = n.subscribe("/joint_states", 1000, &RobotArm::JointStateCallback, &arm);
  
  std::cout << "Executing Dynamic Trajectory..." << std::endl;
  arm.startTrajectory(arm.armExtensionTrajectory());

  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
    ros::spinOnce();
    usleep(50000);
  }
  //clean up
  std::cout << "Clean-up post execution..." << std::endl;
  arm.close_files();
  arm.startTrajectory(arm.armResetTrajectory());
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }

  std::cout<<"Done!"<<std::endl;
}
