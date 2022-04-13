#include <ros/ros.h>
#include "kitting_arm.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "rwa3_arm_test_cpp2");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    KittingArm kitting_arm = KittingArm();
    kitting_arm.print_joints_position(); 

    ROS_INFO("Moving to agv1"); 
    kitting_arm.goToPresetLocation("agv1"); 
    kitting_arm.print_joints_position(); 

    ROS_INFO("Moving to agv2"); 
    kitting_arm.goToPresetLocation("agv2"); 
    kitting_arm.print_joints_position(); 

    ROS_INFO("Moving to agv3"); 
    kitting_arm.goToPresetLocation("agv3"); 
    kitting_arm.print_joints_position(); 

    ROS_INFO("Moving to agv4"); 
    kitting_arm.goToPresetLocation("agv4"); 
    kitting_arm.print_joints_position(); 

    ROS_INFO("Turn to belt"); 
    kitting_arm.turnToBelt(); 
    kitting_arm.print_joints_position(); 
    ROS_INFO("Turn to bins"); 
    kitting_arm.turnToBins(); 
    kitting_arm.print_joints_position(); 
    ROS_INFO("Move to 3"); 
    kitting_arm.moveBaseTo(3); 
    kitting_arm.print_joints_position(); 

    ros::waitForShutdown();
}


