#include <ros/ros.h>
#include "gantry_arm.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "gantry_arm");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    GantryArm gantry_arm = GantryArm();
    // gantry_arm.print_joints_position(); 

    while (ros::ok()) {
      auto success = gantry_arm.get_order(); 

      if (success) {
        gantry_arm.plan(); 
        gantry_arm.execute(); 

      }
      else
        ros::shutdown(); 
    }
   
    ros::waitForShutdown();
}