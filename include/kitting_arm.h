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
#include <queue>
#include <tuple>

// nist
#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>

#include <ariac_group1/PartTask.h>
#include <ariac_group1/PartInfo.h>
#include <ariac_group1/GetParts.h>

// custom
#include "utility.h"
#include "shipments.h"
#include "constants.h"
#include "agv.h"

struct ArmPresetLocation {
      std::vector<double> joints_position;  //9 joints
      std::string name;
};

class KittingArm {
  public: 
    KittingArm();

    bool movePart(const ariac_group1::PartInfo& part_init_info, const ariac_group1::PartTask& part_task); 
    bool pickPart(std::string part_type, const geometry_msgs::Pose& part_init_pose, std::string camera_id, bool flip=false);
    geometry_msgs::Pose placePart(std::string part_type, geometry_msgs::Pose part_init_pose, geometry_msgs::Pose part_goal_pose, std::string agv, bool flip=false);
    void testPreset(const std::vector<ArmPresetLocation>& preset_list);
    bool flip_part(const ariac_group1::PartTask& part_task); 

    void activateGripper();
    void deactivateGripper();
    nist_gear::VacuumGripperState getGripperState();
    bool discard_faulty(const nist_gear::Model& faulty_part, std::string agv_id); 
    bool check_faulty(const nist_gear::Model& faulty_part); 

    

    // Send command message to robot controller
    bool sendJointPosition(trajectory_msgs::JointTrajectory command_msg);
    void goToPresetLocation(std::string location_name);

    bool move_arm_group(); 
    void moveBaseTo(double linear_arm_actuator_joint_position);
    void resetArm(); 
    void turnToBins(); 
    void turnToBelt(); 
    void lift(); 
    void setPickConstraints(); 
    bool moveTargetPose(const geometry_msgs::Pose& pose); 

    void print_joint_group_positions();  
    void print_shipments_total_parts();  

    bool get_order(); 
    void plan(); 
    void execute(); 

    //--preset locations;
    ArmPresetLocation home_face_belt, home_face_bins, 
                      location_belt_part, location_belt_intercept,
                      location_agv1, location_agv2, location_agv3, location_agv4,
                      location_bins0, location_bins1; 

  private:
    // update joint positions
    void copyCurrentJointsPosition(); 

    // callbacks
    void gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& gripper_state_msg);
    void arm_joint_states_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg);
    void arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);
    void part_task_callback(const ariac_group1::PartTask::ConstPtr& msg); 

    bool check_insufficient_shipment(int priority); 
    void clear_part_task(ariac_group1::PartTask& part_task, int& priority); 
    ShipmentState check_shipment_state(ariac_group1::PartTask& part_task, nist_gear::Model& wrong_part); 
    void process_shipment_state(ShipmentState shipment_state, ariac_group1::PartTask& part_task, int& priority, nist_gear::Model& wrong_part); 

    bool check_emergency_interrupt(); 
    void move_to_belt_intercept_pose(const geometry_msgs::Pose& belt_part); 
    bool get_belt_part(double range); 
    void place_to_vacancy(const geometry_msgs::Pose& vacancy_pose, bool from_belt=true); 

    // void submit_shipment(const std::string& agv_id,
                         // const std::string& shipment_type,
                         // const std::string& station_id); 
 

    std::vector<double> m_joint_group_positions;
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

    ros::ServiceClient m_get_parts_client;

    ros::ServiceClient m_is_faulty_client;

    ros::ServiceClient m_parts_in_camera_client;
    ros::ServiceClient m_is_part_picked_client;
    ros::ServiceClient m_get_part_position_client;  
    ros::ServiceClient m_check_quality_sensor_client;  
    ros::ServiceClient m_get_competition_time_client;   
    ros::ServiceClient m_is_belt_sensor_triggered_client;   
    ros::ServiceClient m_get_belt_part_client; 
    ros::ServiceClient m_get_belt_proximity_sensor_client;  
    ros::ServiceClient m_get_vacancy_pose_client;  
    ros::ServiceClient m_parts_under_camera_client;  

    // publishers
    ros::Publisher m_arm_joint_trajectory_publisher;
    // joint states subscribers
    ros::Subscriber m_arm_joint_states_subscriber;
    // controller state subscribers
    ros::Subscriber m_arm_controller_state_subscriber;

    ros::Subscriber m_part_task_subscriber;  


    std::vector<std::string> m_agvs_id = {"agv1", "agv2", "agv3", "agv4"}; 
    std::map<std::string, std::unique_ptr<AGV>> m_agvs_dict; 
    
    Shipments m_shipments; 
    std::vector<std::tuple<int, std::unique_ptr<ariac_group1::PartTask>>> m_part_task_queue; 

    // std::map<std::string, int> m_shipments_total_parts; 

    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 
}; 


#endif
