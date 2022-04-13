#include <ros/ros.h>
#include "kitting_arm.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "kitting_arm");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    KittingArm kitting_arm = KittingArm();
    // kitting_arm.print_joints_position(); 

    while (ros::ok()) {
      auto success = kitting_arm.get_order(); 

      if (success) {
        kitting_arm.plan(); 
        kitting_arm.execute(); 

      }
      else
        ros::shutdown(); 
    }
   
    ros::waitForShutdown();
}


