#include <ros/ros.h>

#include <std_msgs/String.h>
#include "arm.h"
#include "factory_manager.h"

std::string competition_state;  

void competition_state_callback(const std_msgs::String::ConstPtr &msg)
{
  competition_state = msg->data; 
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rwa3_arm_test_cpp");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Subscriber competition_state_subscriber = nh.subscribe("/ariac/competition_state", 10, competition_state_callback); 

    ros::Rate rate(20); 
    FactoryManager group1 = FactoryManager(&nh); 
    ROS_INFO("Initializing..."); 
    ros::Duration(3.0).sleep();

    Arm arm = Arm(&nh);
    arm.init();
    ROS_INFO("Checking for competition state"); 
    ROS_INFO("Testing arm"); 
    arm.goToPresetLocation("home1"); 
    arm.goToPresetLocation("home2");
    std::string part_frame = "logical_camera_bins0_assembly_pump_red_1_frame";
    geometry_msgs::Pose part_pose;
    part_pose.position.x = 0.1;
    part_pose.position.y = 0.1;
    part_pose.position.z = 0.0;
    part_pose.orientation.x = 0.0;
    part_pose.orientation.y = 0.0;
    part_pose.orientation.z = 0.382683;
    part_pose.orientation.w = 0.923879;
    arm.movePart("assembly_pump_red", part_frame, part_pose,"agv1");
    ROS_INFO("END Testing arm"); 
    ros::waitForShutdown();
}