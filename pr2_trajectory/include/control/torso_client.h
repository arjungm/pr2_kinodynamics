//Arjun Menon

#include <ros/ros.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;

class TorsoController{
  private:
    TorsoClient *torso_client_;

  public:
    TorsoController();
    ~TorsoController();
    void up();
    void down();
};
