#include <ros/ros.h>

#include <std_msgs/String.h>

#include "factory_manager.h"

std::string competition_state;  

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_ariac_competitor_node"); 

  ros::NodeHandle nh; 

  ros::Rate rate(20); 
  FactoryManager group1 = FactoryManager(&nh); 

  ros::spin(); 

  return 0; 
}


