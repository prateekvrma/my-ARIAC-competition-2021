#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include "gantry_arm.h"
#include <utility>
#include <string>
int main(int argc, char** argv) {
    ros::init(argc, argv, "test_gantry_arm");
    ros::NodeHandle node;

    // 0 means that the spinner will use as many threads as there are processors on your machine.
    //If you use 3 for example, only 3 threads will be used.
    ros::AsyncSpinner spinner(0);
    spinner.start();

    
    // initialize the gantry
    GantryArm gantry = GantryArm();
    std::pair<std::string, std::string> waypoints;
    gantry.goToPresetLocation("five");
    gantry.goToPresetLocation("as2"); 
    gantry.goToPresetLocation("agv2_at_as2");
    gantry.goToPresetLocation("as4"); 
    gantry.goToPresetLocation("agv4_at_as4");
    gantry.goToPresetLocation("agv3_at_as4");
    gantry.goToPresetLocation("agv3_at_as3");
    gantry.goToPresetLocation("agv1_at_as1");
    gantry.goToPresetLocation("agv1_at_as2");
    ROS_INFO_STREAM("END !!!");
    ros::waitForShutdown();
}