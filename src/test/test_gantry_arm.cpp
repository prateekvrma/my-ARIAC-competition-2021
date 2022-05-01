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
    gantry.goToPresetLocation("at_agv4_at_as3"); 
        
    // if (gantry.pickPart(part_found)) {
    //     // gantry.goToPresetLocation(gantry.after_bin1_);
    //     ROS_INFO_STREAM("part picked");

    // }
    gantry.goToPresetLocation("at_agv4_at_as3");
    gantry.goToPresetLocation("at_as3");
    // if (gantry.placePart(part_in_agv, "agv1")) {
    //         ROS_INFO_STREAM("part placed");
    // }
    gantry.goToPresetLocation("at_agv2_at_as1");
    
    // if (gantry.pickPart(part_found_2)) {
    //     // gantry.goToPresetLocation(gantry.after_bin1_);
    //     ROS_INFO_STREAM("part picked");

    // }
    gantry.goToPresetLocation("at_agv2_at_as1");
    gantry.goToPresetLocation("at_as1");
    // if (gantry.placePart(part_2, "agv1")) {
    //         ROS_INFO_STREAM("part placed");
    // }
    gantry.goToPresetLocation("at_as1");

    ROS_INFO_STREAM("END !!!");
    ros::waitForShutdown();
}