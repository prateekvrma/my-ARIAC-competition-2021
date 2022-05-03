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
    auto init_orientation = Utility::motioncontrol::quaternionFromEuler(0, 0, 0);
    part_init_pose.orientation.x = init_orientation.getX(); 
    part_init_pose.orientation.y = init_orientation.getY(); 
    part_init_pose.orientation.z = init_orientation.getZ(); 
    part_init_pose.orientation.w = init_orientation.getW(); 

    geometry_msgs::Pose part_goal_pose_in_frame; 
    part_goal_pose_in_frame.position.x = 0.1; 
    part_goal_pose_in_frame.position.y = 0.1; 
    part_goal_pose_in_frame.position.z = 0; 

    auto flat_orientation = Utility::motioncontrol::quaternionFromEuler(0, 0, M_PI/4);
    part_goal_pose_in_frame.orientation.x = flat_orientation.getX(); 
    part_goal_pose_in_frame.orientation.y = flat_orientation.getY(); 
    part_goal_pose_in_frame.orientation.z = flat_orientation.getZ(); 
    part_goal_pose_in_frame.orientation.w = flat_orientation.getW(); 
    //
    std::string camera_id = "logical_camera_bins0"; 
    kitting_arm.pick_part("pump", part_init_pose, camera_id); 
    kitting_arm.place_part("pump", part_init_pose, part_goal_pose_in_frame, "agv1"); 

    // ROS_INFO("Moving to agv1"); 
    // kitting_arm.go_to_preset_location("agv1"); 
    // kitting_arm.print_joints_position(); 
    //
    // ROS_INFO("Moving to agv2"); 
    // kitting_arm.go_to_preset_location("agv2"); 
    // kitting_arm.print_joints_position(); 
    //
    // ROS_INFO("Moving to agv3"); 
    // kitting_arm.go_to_preset_location("agv3"); 
    // kitting_arm.print_joints_position(); 
    //
    // ROS_INFO("Moving to agv4"); 
    // kitting_arm.go_to_preset_location("agv4"); 
    // kitting_arm.print_joints_position(); 
    //
    // ROS_INFO("Turn to belt"); 
    // kitting_arm.turnToBelt(); 
    // kitting_arm.print_joints_position(); 
    // ROS_INFO("Turn to bins"); 
    // kitting_arm.turnToBins(); 
    // kitting_arm.print_joints_position(); 
    // ROS_INFO("Move to 3"); 
    // kitting_arm.move_base_to(3); 
    // kitting_arm.print_joints_position(); 
    // ROS_INFO("Move to -3"); 
    // kitting_arm.move_base_to(-3); 
    // kitting_arm.print_joints_position(); 
    ros::waitForShutdown();
}


