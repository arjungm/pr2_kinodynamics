#include "control/torso_client.h"

TorsoController::TorsoController(){
  torso_client_ = new TorsoClient("torso_controller/position_joint_action", true);
  while(!torso_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the torso action server to come up");
  }
}

TorsoController::~TorsoController(){
  delete torso_client_;
}

void TorsoController::up(){
  pr2_controllers_msgs::SingleJointPositionGoal up;
  up.position = 0.195;  //all the way up is 0.2
  up.min_duration = ros::Duration(2.0);
  up.max_velocity = 1.0;
  
  //use action client to send up command
  ROS_INFO("Sending up goal");
  torso_client_->sendGoal(up);
  torso_client_->waitForResult();
}

void TorsoController::down(){
  pr2_controllers_msgs::SingleJointPositionGoal down;
  down.position = 0.0;
  down.min_duration = ros::Duration(2.0);
  down.max_velocity = 1.0;
  
  //use action client to send down command
  ROS_INFO("Sending down goal");
  torso_client_->sendGoal(down);
  torso_client_->waitForResult(ros::Duration());
}

