#include <ros/ros.h>

#include "sensor_manager.h"

int main(int argc, char **argv){

  ros::init(argc, argv, "test_sensor_manager"); 

  ros::NodeHandle nh; 

  SensorManager sensor_manager(&nh); 

  ros::Rate rate(20); 

  while (ros::ok()){
    sensor_manager.update_parts(); 
    sensor_manager.show_database(); 
    sensor_manager.check_blackout(); 
    ros::spinOnce(); 
    rate.sleep(); 
  }


  return 0; 
}


