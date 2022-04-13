#include <ros/ros.h>
#include "kitting_arm.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "rwa3_arm_test_cpp2");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    KittingArm kitting_arm = KittingArm();
    // kitting_arm.print_joints_position(); 

    geometry_msgs::Pose part_init_pose; 
    part_init_pose.position.x = -1.799; 
    part_init_pose.position.y = 3.48; 
    part_init_pose.position.z = 0.78; 

    geometry_msgs::Pose part_goal_pose_in_frame; 
    part_goal_pose_in_frame.position.x = 0.1; 
    part_goal_pose_in_frame.position.y = 0.1; 
    part_goal_pose_in_frame.position.z = 0; 
    // auto flat_orientation = motioncontrol::quaternionFromEuler(0, 0, M_PI/4);
    // part_goal_pose_in_frame.orientation.x = flat_orientation.getX(); 
    // part_goal_pose_in_frame.orientation.y = flat_orientation.getY(); 
    // part_goal_pose_in_frame.orientation.z = flat_orientation.getZ(); 
    // part_goal_pose_in_frame.orientation.w = flat_orientation.getW(); 
    //
    kitting_arm.pickPart("pump", part_init_pose); 
    // kitting_arm.placePart(part_init_pose, part_goal_pose_in_frame, "agv2"); 

    // ROS_INFO("Moving to agv1"); 
    // kitting_arm.goToPresetLocation("agv1"); 
    // kitting_arm.print_joints_position(); 
    //
    ROS_INFO("Moving to agv2"); 
    kitting_arm.goToPresetLocation("agv2"); 
    kitting_arm.print_joints_position(); 
    //
    // ROS_INFO("Moving to agv3"); 
    // kitting_arm.goToPresetLocation("agv3"); 
    // kitting_arm.print_joints_position(); 
    //
    // ROS_INFO("Moving to agv4"); 
    // kitting_arm.goToPresetLocation("agv4"); 
    // kitting_arm.print_joints_position(); 
    //
    // ROS_INFO("Turn to belt"); 
    // kitting_arm.turnToBelt(); 
    // kitting_arm.print_joints_position(); 
    // ROS_INFO("Turn to bins"); 
    // kitting_arm.turnToBins(); 
    // kitting_arm.print_joints_position(); 
    // ROS_INFO("Move to 3"); 
    // kitting_arm.moveBaseTo(3); 
    // kitting_arm.print_joints_position(); 
    // ROS_INFO("Move to -3"); 
    // kitting_arm.moveBaseTo(-3); 
    // kitting_arm.print_joints_position(); 
    ros::waitForShutdown();
}


