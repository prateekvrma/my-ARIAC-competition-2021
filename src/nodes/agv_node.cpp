#include <ros/ros.h>

#include <std_msgs/String.h>

#include "agv.h"



int main(int argc, char **argv)
{

  ros::init(argc, argv, "test_agv"); 

  if (argc != 2) {
    ROS_INFO("Please enter an argument for the name of the agv."); 
    exit(1); 
  }

  ros::NodeHandle nh; 

  AGV agv = AGV(&nh, argv[1]); 

  while (ros::ok()) {
    auto success = agv.get_order(); 

    if (success) 
      agv.plan(); 
    else
      ros::shutdown(); 
    
  }

  return 0; 
}
