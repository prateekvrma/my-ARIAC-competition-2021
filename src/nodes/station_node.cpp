#include <string>
#include <stdlib.h>

#include <ros/ros.h>

#include "station.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "test_station"); 

  ROS_INFO("%d arguments", argc); 
  if (argc != 2) {
    ROS_INFO("Please enter an argument for the name of the station."); 
    exit(1); 
  }

  ros::NodeHandle nh; 

  Station as2 = Station(&nh, argv[1]); 

  while (ros::ok()) {
    auto success = as2.get_order(); 

    if (success)
      as2.plan(); 
    else
      ros::shutdown(); 
  }


  return 0; 
}
