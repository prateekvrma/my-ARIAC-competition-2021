#include<memory>

#include <ros/ros.h>

#include "sensors.h"


int main(int argc, char **argv){

  ros::init(argc, argv, "test_sensors"); 

  ros::NodeHandle nh; 

  auto logical_camera_bins0 =  std::make_unique<LogicalCamera>(&nh, "logical_camera_bins0"); 
  auto logical_camera_station2 =  std::make_unique<LogicalCamera>(&nh, "logical_camera_station2"); 
  // auto depth_camera_bins1 =  std::make_unique<DepthCamera>(&nh, "depth_camera_bins1"); 
  // auto proximity_sensor_0 =  std::make_unique<ProximitySensor>(&nh, "proximity_sensor_0"); 
  // auto laser_profiler_0 =  std::make_unique<LaserProfiler>(&nh, "laser_profiler_0"); 
  // auto breakbeam_0 =  std::make_unique<BreakBeam>(&nh, "breakbeam_0"); 
  ros::spin(); 

  return 0; 
}
