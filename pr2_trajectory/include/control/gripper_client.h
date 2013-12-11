//author: Arjun G Menon
//email:  agmenon@andrew.cmu.edu OR ajn.menon@gmail.com

#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>

class Gripper{
  typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
  private:
    GripperClient* r_gripper_client_;
    GripperClient* l_gripper_client_;
  public:
    Gripper();
    ~Gripper();
    void open();
    void close(double position, double effort);
    void open_left();
    void close_left(double position, double effort);
    void open_right();
    void close_right(double position, double effort);
};
