//author: Arjun G Menon
//email:  agmenon@andrew.cmu.edu OR ajn.menon@gmail.com
#ifndef DYN_MOD_H
#define DYN_MOD_H

#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <boost/shared_ptr.hpp>
#include <ecl/containers.hpp>
#include <ecl/geometry.hpp>

struct StateDerivative{
  const std::vector<double> fap_;
  StateDerivative(const std::vector<double> &fap) : fap_(fap) {}
  void operator() (const std::vector<double> &y, std::vector<double> &y_prime, const double /*t*/)
  {
    //derivative of [x x'] is [x' x"] 
    y_prime[0] = y[7];
    y_prime[1] = y[8];
    y_prime[2] = y[9];
    y_prime[3] = y[10];
    y_prime[4] = y[11];
    y_prime[5] = y[12];
    y_prime[6] = y[13];

    y_prime[7] = fap_[0];
    y_prime[8] = fap_[1];
    y_prime[9] = fap_[2];
    y_prime[10] = fap_[3];
    y_prime[11] = fap_[4];
    y_prime[12] = fap_[5];
    y_prime[13] = fap_[6];
  }
};

class DynamicsModel{
  public:
    //constructors 
    DynamicsModel();
    DynamicsModel(boost::shared_ptr<KDL::Chain> kdl_chain_ptr, boost::shared_ptr<KDL::Vector> grav_ptr);
    ~DynamicsModel();
    
    //setters
    void setTorqueLimits(std::vector<double> torque_limits);
    void setVerbose(bool enable);
    void setIntegrationDuration(double new_integration_duration_);
    
    //getters
    std::vector<double> getTorqueLimits();
    double getIntegrationDuration();
    
    //joint torques
    void computeJointTorques( const std::vector<double> &q, 
                              const std::vector<double> &qdot, 
                              const std::vector<double> &qdotdot, 
                              std::vector<double> &torques);

    //dynamic parameters of a manipulator
    void getMassMatrix(const std::vector<double> &q_, Eigen::MatrixXd &Mass);
    void getCoriolisMatrix(const std::vector<double> &q_, const std::vector<double> &qdot_, Eigen::MatrixXd &Coriolis);
    void getGravityMatrix(const std::vector<double> &q_, Eigen::MatrixXd &Gravity);

    //integration
    void boost_integrate(const std::vector<double> &q_qdot_in, const std::vector<double> &qdotdot_in, std::vector<double> &q_qdot_out);

    //utility
    bool checkJointTorqueLimits(KDL::JntArray &input_torques);
    bool checkJointTorqueLimits(std::vector<double> &input_torques);
    bool checkJointTorqueLimitsImplicit(std::vector<double> &q0, 
                                        std::vector<double> &q1, 
                                        std::vector<double> &qdot0, 
                                        std::vector<double> &qdot1, 
                                        double t);
    
    //spline based
    void queryTrajectorySplines(const double time, std::vector<double> &q, std::vector<double> &qdot, std::vector<double> &qdotdot);
    void setTrajectorySplines(const std::vector<std::vector<double> > &trajectory);

  private:
    //ptrs
    boost::shared_ptr<KDL::Chain> kdl_chain_ptr_;
    boost::shared_ptr<KDL::Vector> grav_ptr_;
    boost::shared_ptr<KDL::ChainDynParam> cdp_ptr;
    boost::shared_ptr<KDL::ChainIdSolver_RNE> idsolver_ptr;
    //members
    double integration_duration_;
    bool verbose_;
    bool limits_set_;
    std::vector<double> torque_limits_;
    std::vector<ecl::CubicSpline> splines_;
    std::vector<double> fixed_acceleration_profile_;
};
#endif
