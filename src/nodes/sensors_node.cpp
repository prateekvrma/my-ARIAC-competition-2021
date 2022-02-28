#include<memory>

#include <ros/ros.h>

#include <std_msgs/String.h>

#include "sensors.h"

std::string competition_state;  

void competition_state_callback(const std_msgs::String::ConstPtr &msg)
{
  competition_state = msg->data; 
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "test_sensors"); 

  ros::NodeHandle nh; 

  ros::Subscriber competition_state_subscriber = nh.subscribe("/ariac/competition_state", 10, competition_state_callback); 

  auto logical_camera_bins0 =  std::make_unique<LogicalCamera>(&nh, "logical_camera_bins0"); 
  auto depth_camera_bins1 =  std::make_unique<DepthCamera>(&nh, "depth_camera_bins1"); 
  auto proximity_sensor_0 =  std::make_unique<ProximitySensor>(&nh, "proximity_sensor_0"); 
  auto laser_profiler_0 =  std::make_unique<LaserProfiler>(&nh, "laser_profiler_0"); 
  auto breakbeam_0 =  std::make_unique<BreakBeam>(&nh, "breakbeam_0"); 

  ros::Rate rate(20); 
  while (ros::ok()) {
    if (competition_state=="done") {
      ros::shutdown(); 
    }
    ros::spinOnce(); 
    rate.sleep(); 
  }

  return 0; 
}
