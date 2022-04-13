#include <ros/ros.h>
#include "arm.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "rwa3_arm_test_cpp2");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    Arm arm = Arm(&nh);
    arm.init();
    ros::waitForShutdown();
}


