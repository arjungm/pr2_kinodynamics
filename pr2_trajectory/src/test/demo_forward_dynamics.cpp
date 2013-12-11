//author: Arjun G Menon
//email:  agmenon@andrew.cmu.edu OR ajn.menon@gmail.com
#include <dynamics_model.h> //now in a seperate package
#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <boost/make_shared.hpp>

void print_stl_vec( std::string name, std::vector<double> vec)
{
  std::cout << name << ": [ ";
  for(size_t i=0; i<vec.size(); i++)
    std::cout << vec.at(i) << " ";
  std::cout << "]" << std::endl;
}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh("~");
  
  //get robot state
  std::cout << "Getting the urdf..." << std::endl;
  std::string robot_description;
  std::string robot_param;
  
  nh.searchParam("robot_description",robot_param);
  nh.param<std::string>(robot_param,robot_description,"");
  
  //check the param server return
  if(robot_description.empty())
  {
    std::cout << "Failed to retreive robot urdf" << std::endl;
    return 0;
  }

  //parse the KDL tree
  KDL::Tree kdl_tree; 
  if (!kdl_parser::treeFromString(robot_description, kdl_tree))
  {
    std::cout << "Failed to parse the kdl tree" << std::endl;
    return 0;
  }

  //parse the KDL chain
  KDL::Chain planning_chain;
  if (!kdl_tree.getChain("torso_lift_link", "r_wrist_roll_link", planning_chain))
  {
    std::cout << "Failed to parse the kdl chain" << std::endl;
    return 0;
  }
  boost::shared_ptr<KDL::Chain> planning_chain_ = boost::make_shared<KDL::Chain>(planning_chain);
  std::cout << "KDL chain has " << planning_chain_->getNrOfSegments() << " segments and " << planning_chain_->getNrOfJoints() << " joints." << std::endl;
  
  //construct a dynamics model
  boost::shared_ptr<KDL::Vector> grav_ = boost::make_shared<KDL::Vector>(0.0, 0.0, 9.8);
  boost::shared_ptr<DynamicsModel> dm_ = boost::make_shared<DynamicsModel>( planning_chain_ ,grav_ );
  dm_->setIntegrationDuration(0.5);
  
  std::vector<double> state_in(14,0.0);
  std::vector<double> accel_profile(7,0.0); accel_profile[0] = 1.0;
  std::vector<double> state_out1(14,0.0);
  std::vector<double> state_out2(14,0.0);
  dm_->boost_integrate(state_in, accel_profile, state_out1);
  std::vector<double> accel_profile2(7,0.0); accel_profile2[1] = 1.0;
  dm_->boost_integrate(state_out1, accel_profile2, state_out2);

  
  print_stl_vec("initial",state_in);
  print_stl_vec("final", state_out2);

  std::cout << "Done!" << std::endl;
}
