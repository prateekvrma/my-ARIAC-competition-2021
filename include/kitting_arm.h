#ifndef KITTING_ARM_H
#define KITTING_ARM_H

// ros
#include <ros/ros.h>
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

// services and messages
#include <std_msgs/String.h>
// nist
#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>

// custom
#include <ariac_group1/PartTask.h>
#include <ariac_group1/PartInfo.h>
#include <ariac_group1/GetParts.h>
#include <ariac_group1/GetWorkingStation.h>


// custom library
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

    // flow control
    void wait_for_belt(int wait_time=5); 
    bool get_order(); 
    void plan(); 
    void execute(); 

    // basic actions
    void go_to_preset_location(std::string location_name);
    void move_base_to(double linear_arm_actuator_joint_position);
    void reset_arm(); 
    bool move_part(const ariac_group1::PartInfo& part_init_info, const ariac_group1::PartTask& part_task); 
    bool move_back_row_part(const ariac_group1::PartInfo& part_init_info, const ariac_group1::PartTask& part_task); 
    bool pick_part(std::string part_type, const geometry_msgs::Pose& part_init_pose, std::string camera_id, bool flip=false);
    bool pick_back_row_part(std::string part_type, const geometry_msgs::Pose& part_init_pose, std::string camera_id);
    geometry_msgs::Pose place_part(std::string part_type, geometry_msgs::Pose part_init_pose, geometry_msgs::Pose part_goal_pose, std::string agv, bool flip=false);
    bool discard_faulty(const nist_gear::Model& faulty_part, std::string agv_id); 
    bool flip_part(const ariac_group1::PartTask& part_task); 
    void lift(); 
    void turn_to_bins(); 
    void turn_to_belt(); 

    // belt actions
    void move_to_belt_intercept_pose(const geometry_msgs::Pose& belt_part); 
    bool get_belt_part(double range); 
    void place_to_vacancy(const geometry_msgs::Pose& vacancy_pose, bool from_belt=true); 

  private:
    // callbacks
    void arm_joint_states_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg);
    void arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);
    void gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& gripper_state_msg);

    // Shipments and part tasks
    bool check_insufficient_shipment(int priority); 
    void clear_part_task(ariac_group1::PartTask& part_task, int& priority); 
    ShipmentState check_shipment_state(ariac_group1::PartTask& part_task, nist_gear::Model& wrong_part); 
    void process_shipment_state(ShipmentState shipment_state, ariac_group1::PartTask& part_task, int& priority, nist_gear::Model& wrong_part); 

    bool check_emergency_interrupt(); 

    // arm control 
    void copy_current_joints_position(); 
    bool move_arm_group(); 

    // gripper control
    void activate_gripper();
    void deactivate_gripper();
    nist_gear::VacuumGripperState get_gripper_state();

    // auxiliary functions
    bool move_target_pose(const geometry_msgs::Pose& pose); 
    bool check_faulty(const nist_gear::Model& faulty_part); 
    void set_pick_constraints(); 

    // utils
    void print_joint_group_positions();  

    bool get_working_station(ariac_group1::GetWorkingStation::Request &req,
                             ariac_group1::GetWorkingStation::Response &res); 

    // ros
    ros::NodeHandle m_nh;

    // arm control
    std::vector<double> m_joint_group_positions;
    std::string m_planning_group;
    moveit::planning_interface::MoveGroupInterface::Options m_arm_options;
    moveit::planning_interface::MoveGroupInterface m_arm_group;
    sensor_msgs::JointState m_current_joint_states;
    control_msgs::JointTrajectoryControllerState m_arm_controller_state;

    // gripper control
    nist_gear::VacuumGripperState m_gripper_state;

    // ros publisher
    ros::Publisher m_arm_joint_trajectory_publisher;

    // ros subscriber
    ros::Subscriber m_gripper_state_subscriber;
    ros::Subscriber m_arm_joint_states_subscriber;
    ros::Subscriber m_arm_controller_state_subscriber;

    // ros service server
    ros::ServiceServer m_get_working_station_service; 

    // ros service client
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

    // preset locations;
    ArmPresetLocation home_face_belt, home_face_bins, 
                      location_belt_part, location_belt_intercept,
                      location_agv1, location_agv2, location_agv3, location_agv4,
                      location_bins0, location_bins1; 

    // AGVs
    std::vector<std::string> m_agvs_id = {"agv1", "agv2", "agv3", "agv4"}; 
    std::map<std::string, std::unique_ptr<AGV>> m_agvs_dict; 
    
    // Shipments and part tasks
    Shipments m_shipments; 
    std::vector<std::tuple<int, std::unique_ptr<ariac_group1::PartTask>>> m_part_task_queue; 

    // current arm working station
    std::string m_working_station = "home"; 

    std::unique_ptr<std::mutex> m_mutex_ptr = std::make_unique<std::mutex>(); 
}; 


#endif
