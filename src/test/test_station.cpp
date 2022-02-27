#include <string>

#include <ros/ros.h>

#include "station.h"


int main(int argc, char **argv){

  ros::init(argc, argv, "test_station"); 

  ros::NodeHandle nh; 

  Station as2 = Station(&nh, "as2"); 

  while(ros::ok()){
    auto success = as2.get_order(); 
    if(success){
      as2.plan(); 
    }else{
      ros::shutdown(); 
    }
  }


  return 0; 
}
