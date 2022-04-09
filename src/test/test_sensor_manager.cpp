#include <ros/ros.h>

#include "sensor_manager.h"

int main(int argc, char **argv){

  ros::init(argc, argv, "test_sensor_manager"); 

  ros::NodeHandle nh; 

  SensorManager sensor_manager(&nh); 

  ros::spin(); 

  return 0; 
}


