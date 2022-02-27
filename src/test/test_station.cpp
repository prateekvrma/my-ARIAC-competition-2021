#include <string>

#include <ros/ros.h>

#include "station.h"


int main(int argc, char **argv){

  ros::init(argc, argv, "test_station"); 

  ros::NodeHandle nh; 

  Station as2 = Station(&nh, "as2"); 

  as2.plan(); 

  return 0; 
}
