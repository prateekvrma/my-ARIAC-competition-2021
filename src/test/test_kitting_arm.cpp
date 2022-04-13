#include <ros/ros.h>
#include "kitting_arm.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "rwa3_arm_test_cpp2");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    KittingArm kitting_arm = KittingArm();
    kitting_arm.print_joints_position(); 
    // arm.init();
    // ROS_INFO("Moving to home1"); 
    // arm.goToPresetLocation("home1"); 
    // ROS_INFO("Moving to home2"); 
    // arm.goToPresetLocation("home2"); 
    // ROS_INFO("Moving to home1"); 
    // arm.goToPresetLocation("home1"); 
    // ros::waitForShutdown();
}


