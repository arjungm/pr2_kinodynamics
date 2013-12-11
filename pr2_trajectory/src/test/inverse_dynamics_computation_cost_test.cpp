//author: Arjun G Menon
//email:  agmenon@andrew.cmu.edu OR ajn.menon@gmail.com
#include <ros/ros.h>
#include <pr2_mechanism_model/chain.h>
#include <pr2_mechanism_model/robot.h>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <time.h>

double frand(double fmax, double fmin)
{
  return fmin+((double)rand()/RAND_MAX)*(fmax-fmin);
}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");

  // Start the trajectory
  ros::NodeHandle n;

  //get robot state
  std::string urdf_param_name="robot_description";
  std::string urdf_string;
  n.getParam(urdf_param_name,urdf_string);

  std::string root_name = "torso_lift_link";
  std::string tip_name = "r_wrist_roll_link";
  KDL::Tree modeltree;
  if (!kdl_parser::treeFromString(urdf_string, modeltree))
    ROS_ERROR("Failed to construct kdl tree");

  KDL::Chain kdl_chain;
  if (!modeltree.getChain(root_name,tip_name,kdl_chain))
    ROS_ERROR("could not find a chain from '%s' to '%s'",root_name.c_str(), tip_name.c_str());

  KDL::Vector grav(0.0,0.0,-9.81);
  
  KDL::JntArray q(kdl_chain.getNrOfJoints());
  KDL::JntArray qdot(kdl_chain.getNrOfJoints());
  KDL::JntArray qdotdot(kdl_chain.getNrOfJoints());
  KDL::Wrenches f(kdl_chain.getNrOfJoints());
  KDL::JntArray tau(kdl_chain.getNrOfJoints());
  
  KDL::ChainIdSolver_RNE idsolver(kdl_chain, grav);
  
  //loop over solving for the inverse dynamics
  double time_total=0;
  double iterations=1000000;

  clock_t start;
  for(int i=0; i<iterations; i++){
    start = clock();
    idsolver.CartToJnt(q, qdot, qdotdot, f, tau);
    time_total += (clock()-start);

    //modify q, qdot, qdotdot
    srand(time(NULL));
    double factor = 3.14/180;
    q.data(0,0) = factor*(frand(40,-130));
    q.data(1,0) = factor*(frand(80,-30));
    q.data(2,0) = factor*(frand(44,-224));
    q.data(3,0) = factor*(frand(133,0));
    q.data(4,0) = factor*(frand(360,0));
    q.data(5,0) = factor*(frand(130,0));
    q.data(6,0) = factor*(frand(360,0));

    qdot.data(0,0) = frand(2.10,0);
    qdot.data(1,0) = frand(2.10,0);
    qdot.data(2,0) = frand(3.27,0);
    qdot.data(3,0) = frand(3.30,0);
    qdot.data(4,0) = frand(3.60,0);
    qdot.data(5,0) = frand(3.10,0);
    qdot.data(6,0) = frand(3.60,0);
  }
  
  std::cout << "Cost:" << time_total/(iterations*CLOCKS_PER_SEC) << std::endl;
}

