//author: Arjun G Menon
//email:  agmenon@andrew.cmu.edu OR ajn.menon@gmail.com

#include "dynamics_model.h"

# include <cstdlib>
# include <iostream>
# include <iomanip>
# include <cmath>
# include <ctime>

#include <Eigen/Core>
#include <Eigen/LU>

#include <boost/make_shared.hpp>
#include <boost/array.hpp>
#include </home/agmenon/ros_workspace/code/boost_install/include/boost/numeric/odeint.hpp>

DynamicsModel::DynamicsModel() : integration_duration_(0.0) {}

DynamicsModel::DynamicsModel(boost::shared_ptr<KDL::Chain> kdl_chain_ptr, boost::shared_ptr<KDL::Vector> grav_ptr) 
                              : kdl_chain_ptr_(kdl_chain_ptr), 
                                grav_ptr_(grav_ptr),
                                cdp_ptr(new KDL::ChainDynParam(*kdl_chain_ptr_, *grav_ptr_)),
                                idsolver_ptr(new KDL::ChainIdSolver_RNE(*kdl_chain_ptr_, *grav_ptr_))
{
  verbose_ = false; 
  limits_set_ = false;
}

DynamicsModel::~DynamicsModel() {}

void DynamicsModel::setVerbose(bool enable){ verbose_ = enable; }
void DynamicsModel::setIntegrationDuration(double new_integration_duration_){ integration_duration_ = new_integration_duration_; }
void DynamicsModel::setTorqueLimits(std::vector<double> torque_limits){
  torque_limits_ = torque_limits;
  limits_set_ = true;
}

std::vector<double> DynamicsModel::getTorqueLimits() { 
  if(limits_set_)
    return torque_limits_;
  else{
    torque_limits_.resize(0);
    return torque_limits_;
  }
}
double DynamicsModel::getIntegrationDuration(){ return integration_duration_; } 


void DynamicsModel::boost_integrate(const std::vector<double> &q_qdot_in, const std::vector<double> &qdotdot_in, std::vector<double> &q_qdot_out){
  std::vector<double> state( q_qdot_in.begin(), q_qdot_in.end());
  //set the fixed acceleration profile
  StateDerivative deriv_fun( qdotdot_in );
  //use odeint
  size_t integration_steps = boost::numeric::odeint::integrate( deriv_fun, state, 0.0, integration_duration_, 0.01 );
  
  std::cout << "... integration took " << integration_steps << " steps" << std::endl;

  //state is rewritten with the output state
  q_qdot_out = state;
}

void DynamicsModel::getMassMatrix(const std::vector<double> &q_, Eigen::MatrixXd &Mass)
{
  KDL::JntArray q(kdl_chain_ptr_->getNrOfJoints());
  for(size_t i=0; i<kdl_chain_ptr_->getNrOfJoints(); i++)
    q.data(i,0) = q_.at(i);
  KDL::JntSpaceInertiaMatrix M(kdl_chain_ptr_->getNrOfJoints());
  cdp_ptr->JntToMass(q, M);
  Mass=Eigen::MatrixXd::Identity(M.data.rows(),M.data.cols());
  for(int i=0; i<M.data.rows(); i++)
  {
    for(int j=0; j<M.data.cols(); j++)
    {
      Mass(i,j)=M.data(i,j);
    }
  }
}

void DynamicsModel::getCoriolisMatrix(const std::vector<double> &q_, const std::vector<double> &qdot_, Eigen::MatrixXd &Coriolis)
{
  KDL::JntArray q(kdl_chain_ptr_->getNrOfJoints());
  KDL::JntArray qdot(kdl_chain_ptr_->getNrOfJoints());
  for(size_t i=0; i<kdl_chain_ptr_->getNrOfJoints(); i++){
    q.data(i,0) = q_.at(i);
    qdot.data(i,0) = qdot_.at(i);
  }
  KDL::JntArray C(kdl_chain_ptr_->getNrOfJoints());
  cdp_ptr->JntToCoriolis(q, qdot, C);
  Coriolis=Eigen::MatrixXd::Identity(C.data.rows(),C.data.cols());
  for(int i=0; i<C.data.rows(); i++)
  {
    for(int j=0; j<C.data.cols(); j++)
    {
      Coriolis(i,j)=C.data(i,j);
    }
  }
}

void DynamicsModel::getGravityMatrix(const std::vector<double> &q_, Eigen::MatrixXd &Gravity)
{
  KDL::JntArray q(kdl_chain_ptr_->getNrOfJoints());
  for(size_t i=0; i<kdl_chain_ptr_->getNrOfJoints(); i++)
    q.data(i,0) = q_.at(i);
  KDL::JntArray G(kdl_chain_ptr_->getNrOfJoints());
  cdp_ptr->JntToGravity(q, G);
  Gravity=Eigen::MatrixXd::Identity(G.data.rows(),G.data.cols());
  for(int i=0; i<G.data.rows(); i++)
  {
    for(int j=0; j<G.data.cols(); j++)
    {
      Gravity(i,j)=G.data(i,j);
    }
  }
}

void DynamicsModel::computeJointTorques(const std::vector<double> &q, 
                                          const std::vector<double> &qdot, 
                                          const std::vector<double> &qdotdot_in, 
                                          std::vector<double> &torques)
{
  //TODO assert the dimensions are correct 

  KDL::JntArray kdl_q(kdl_chain_ptr_->getNrOfJoints());
  KDL::JntArray kdl_qdot(kdl_chain_ptr_->getNrOfJoints());
  KDL::JntArray kdl_qdotdot(kdl_chain_ptr_->getNrOfJoints());
  
  //dynamic parameters for computing joint torque
  KDL::JntSpaceInertiaMatrix kdl_M(kdl_chain_ptr_->getNrOfJoints());
  KDL::JntArray kdl_C(kdl_chain_ptr_->getNrOfJoints());
  KDL::JntArray kdl_G(kdl_chain_ptr_->getNrOfJoints());
  KDL::JntArray kdl_tauMCG(kdl_chain_ptr_->getNrOfJoints());

  KDL::SetToZero(kdl_q);
  KDL::SetToZero(kdl_qdot);
  KDL::SetToZero(kdl_qdotdot);

  for(size_t i=0; i<kdl_q.rows(); i++)
  {
    kdl_q(i)=q[i];
    kdl_qdot(i)=qdot[i];
    kdl_qdotdot(i)=qdotdot_in[i];
  }

  cdp_ptr->JntToMass(kdl_q, kdl_M);
  cdp_ptr->JntToCoriolis(kdl_q, kdl_qdot, kdl_C);
  cdp_ptr->JntToGravity(kdl_q, kdl_G);
  
  //M * qdd + C + G = Tau
  kdl_tauMCG.data = kdl_M.data.lazyProduct(kdl_qdotdot.data) + kdl_C.data + kdl_G.data;
  
  //set return container
  torques.resize(kdl_tauMCG.rows());
  for(size_t i=0; i<kdl_tauMCG.rows(); i++)
    torques[i] = kdl_tauMCG(i);
}

bool DynamicsModel::checkJointTorqueLimits(KDL::JntArray& input_torques)
{
  if(limits_set_)
  {
    if(input_torques.rows() != torque_limits_.size()){
      //TODO make assert
      //printf("Incorrect number of input torques for checking. Expected: %d ; Received: %d", (int)torque_limits_.size(), (int)input_torques.data.rows());
    }
    
    for(size_t i=0; i<torque_limits_.size(); i++)
    {
      if(fabs(input_torques.data(i,0)) > fabs(torque_limits_.at(i)))
      {
        return false;
      }
    }
    return true;
  }
  return true;
}

bool DynamicsModel::checkJointTorqueLimits(std::vector<double> &input_torques)
{
  if(limits_set_)
  {
    if(input_torques.size() != torque_limits_.size()){
      //printf("Incorrect number of input torques for checking. Expected: %d ; Received: %d", (int)torque_limits_.size(), (int)input_torques.data.rows());
    }
    
    for(size_t i=0; i<torque_limits_.size(); i++)
    {
      if(fabs(input_torques.at(i)) > fabs(torque_limits_.at(i)))
      {
        //std::cout << "ind: " << i <<  " applied: " << input_torques.at(i) << " limit: " << torque_limits_.at(i) << std::endl;
        return false;
      }
    }
    return true;
  }
  return true;
}

bool DynamicsModel::checkJointTorqueLimitsImplicit(std::vector<double> &q0, std::vector<double> &q1, std::vector<double> &qdot0, std::vector<double> &qdot1, double t){
  
  std::vector<double> qdotdot_average( q0.size(), 0.0);
  //find the average joint acceleration from the values of the joint velocities and the time duration t
  for(size_t i=0; i<q0.size(); i++)
  {
    qdotdot_average.at(i) = (qdot1.at(i) - qdot0.at(i))/t;
  }

  //find the required torques
  std::vector<double> torques;
  computeJointTorques(q0, qdot0, qdotdot_average, torques);
  
  return checkJointTorqueLimits(torques);
}

void DynamicsModel::setTrajectorySplines(const std::vector<std::vector<double> > &trajectory)
{
  splines_.clear();
  int n = trajectory.size();
  for(int joint=0; joint<7; joint++)
  {
    ecl::Array<double> x_set(n);
    ecl::Array<double> y_set(n);
    for(int i = 0; i < n; i++)
    {
      x_set.at(i)=trajectory.at(i).at(14);
      y_set.at(i)=trajectory.at(i).at(joint);
    }
    ecl::CubicSpline cubic;
    cubic = ecl::CubicSpline::ContinuousDerivatives(x_set, y_set, trajectory.at(0).at(joint+7), trajectory.at(n-1).at(joint+7));
    splines_.push_back(cubic);
  }
}

void DynamicsModel::queryTrajectorySplines(const double time, std::vector<double> &q, std::vector<double> &qdot, std::vector<double> &qdotdot)
{
  q.resize(7);
  qdot.resize(7);
  qdotdot.resize(7);
  ecl::CubicSpline cubic;
  for(int joint = 0; joint < 7; joint ++)
  {
    cubic = splines_.at(joint);
    q.at(joint) = cubic(time);
    qdot.at(joint) = cubic.derivative(time);
    qdotdot.at(joint) = cubic.dderivative(time);
  }
}
