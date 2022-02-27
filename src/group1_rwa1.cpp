#include <ros/ros.h>

#include <std_msgs/String.h>

#include "factory_manager.h"

std::string competition_state;  

void competition_state_callback(const std_msgs::String::ConstPtr &msg){
  competition_state = msg->data; 
}

int main(int argc, char **argv){
  ros::init(argc, argv, "my_ariac_competitor_node"); 

  ros::NodeHandle nh; 

  ros::Subscriber competition_state_subscriber = nh.subscribe("/ariac/competition_state", 10, competition_state_callback); 

  ros::Rate rate(20); 
  FactoryManager group1 = FactoryManager(&nh); 
  ROS_INFO("Checking for competition state"); 

  while(ros::ok()){
    if (competition_state=="init"){
      group1.start_competition(); 

      ROS_INFO("Wait for a 10s..."); 
      ros::Rate wait_rate(1); 
      int count = 10; 
      while(count --> 0){
        ROS_INFO("Count down: %d", count); 
        ros::spinOnce(); 
        wait_rate.sleep(); 
      }

      group1.plan(); 
      group1.end_competition(); 

    } 
    else if (competition_state=="done"){
      ros::shutdown(); 
    }

    ros::spinOnce(); 
    rate.sleep(); 
  }

  return 0; 
}


