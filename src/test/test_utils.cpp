#include "utility.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_utils"); 
  ros::NodeHandle nh; 

  geometry_msgs::Pose pose; 
  pose.position.x = -2.01 ; 
  pose.position.y = 4.575 ; 
  pose.position.z = 0.7811; 
    tf2::Quaternion quaternionFromEuler();  
  pose.orientation.x = 0; 
  pose.orientation.y = 0; 
  pose.orientation.z = 0; 
  pose.orientation.w = 1; 
  auto t_pose = Utility::motioncontrol::transformToTrayFrame(pose, "agv1"); 
  Utility::motioncontrol::print(t_pose); 
  return 0; 
}
