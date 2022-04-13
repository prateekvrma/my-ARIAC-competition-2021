#ifndef KITTING_ARM_H
#define KITTING_ARM_H

// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
// standard library
#include <string>
#include <vector>
#include <array>
#include <cstdarg>
#include <memory>
#include <mutex>

// nist
#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>

#include <ariac_group1/PartTask.h>

// custom
#include "utils.h"
#include "utility.h"

struct ArmPresetLocation {
      std::vector<double> joints_pos;  //9 joints
      std::string name;
};

class KittingArm {
  public: 
    KittingArm();

    // bool pickPart(std::string part_type, geometry_msgs::Pose part_pose);
    // bool placePart(geometry_msgs::Pose part_init_pose, geometry_msgs::Pose part_goal_pose, std::string agv);
    void testPreset(const std::vector<ArmPresetLocation>& preset_list);
    // void movePart(std::string part_type, std::string camera_frame, geometry_msgs::Pose goal_in_tray_frame, std::string agv);
    // void activateGripper();
    // void deactivateGripper();

    void moveBaseTo(double linear_arm_actuator_joint_position);
    nist_gear::VacuumGripperState getGripperState();

    

    // Send command message to robot controller
    bool sendJointPosition(trajectory_msgs::JointTrajectory command_msg);
    void goToPresetLocation(std::string location_name);

    void print_joints_position();  

    //--preset locations;
    ArmPresetLocation home_face_belt, home_face_bins,
                      location_agv1, location_agv2, location_agv3, location_agv4;

  private:
    // callbacks
    void gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& gripper_state_msg);
    void arm_joint_states_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg);
    void arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);
    void part_task_callback(const ariac_group1::PartTask::ConstPtr& msg); 

    std::vector<double> m_joint_group_positions;
    std::vector<double> m_joint_arm_positions;
    ros::NodeHandle m_nh;
    std::string m_planning_group;
    moveit::planning_interface::MoveGroupInterface::Options m_arm_options;
    moveit::planning_interface::MoveGroupInterface m_arm_group;
    sensor_msgs::JointState m_current_joint_states;
    control_msgs::JointTrajectoryControllerState m_arm_controller_state;

    nist_gear::VacuumGripperState m_gripper_state;
    // gripper state subscriber
    ros::Subscriber m_gripper_state_subscriber;
    // service client
    ros::ServiceClient m_gripper_control_client;
    // publishers
    ros::Publisher m_arm_joint_trajectory_publisher;
    // joint states subscribers
    ros::Subscriber m_arm_joint_states_subscriber;
    // controller state subscribers
    ros::Subscriber m_arm_controller_state_subscriber;

    ros::Subscriber m_part_task_subscriber;  

    std::vector<std::unique_ptr<nist_gear::Product>> m_part_task_queue; 

    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 
}; 


#endif
