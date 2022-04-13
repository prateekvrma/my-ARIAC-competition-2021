#include "kitting_arm.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

KittingArm::KittingArm():
  m_nh{"/ariac/kitting"},
  m_planning_group{"/ariac/kitting/robot_description"},
  m_arm_options{"kitting_arm", m_planning_group, m_nh},
  m_arm_group{m_arm_options} 
{
  // make sure the planning group operates in the world frame
  // check the name of the end effector
  // ROS_INFO_NAMED("init", "End effector link: %s", arm_group_.getEndEffectorLink().c_str());


  // publishers to directly control the joints without moveit
  m_arm_joint_trajectory_publisher =
      m_nh.advertise<trajectory_msgs::JointTrajectory>("/ariac/kitting/kitting_arm_controller/command", 10);
  // joint state subscribers
  m_arm_joint_states_subscriber =
      m_nh.subscribe("/ariac/kitting/joint_states", 10, &KittingArm::arm_joint_states_callback, this);
  // controller state subscribers
  m_arm_controller_state_subscriber =
      m_nh.subscribe("/ariac/kitting/kitting_arm_controller/state", 10, &KittingArm::arm_controller_state_callback, this);
  // gripper state subscriber
  m_gripper_state_subscriber = 
      m_nh.subscribe("/ariac/kitting/arm/gripper/state", 10, &KittingArm::gripper_state_callback, this);
  // controller state subscribers
  m_gripper_control_client =
      m_nh.serviceClient<nist_gear::VacuumGripperControl>("/ariac/kitting/arm/gripper/control");
  m_gripper_control_client.waitForExistence();

  // part task subscriber
  m_part_task_subscriber =
      m_nh.subscribe("/part_task", 10, &KittingArm::part_task_callback, this); 


  // Preset locations
  // ^^^^^^^^^^^^^^^^
  // Joints for the arm are in this order:
  // - linear_arm_actuator_joint
  // - shoulder_pan_joint
  // - shoulder_lift_joint
  // - elbow_joint
  // - wrist_1_joint
  // - wrist_2_joint
  // - wrist_3_joint

  double linear_arm_actuator_joint{ 0 };
  double shoulder_pan_joint{ 0 };
  double shoulder_lift_joint{ -1.25 };
  double elbow_joint{ 1.74 };
  double wrist_1_joint{ -2.04 };
  double wrist_2_joint{ -1.57 };
  double wrist_3_joint{ 0 };

  std::vector<double> default_pos = {linear_arm_actuator_joint,
                                     shoulder_pan_joint,
                                     shoulder_lift_joint,
                                     elbow_joint,
                                     wrist_1_joint,
                                     wrist_2_joint,
                                     wrist_3_joint}; 
  //home position
  home_face_belt.joints_position = default_pos;
  home_face_belt.name = "home_face_belt";

  home_face_bins.joints_position = default_pos;
  // shoulder_pan_joint
  home_face_bins.joints_position.at(1) = -M_PI;
  home_face_bins.name = "home_face_bins";

  location_agv1.joints_position = home_face_bins.joints_position; 
  // linear actuator at y axis 
  location_agv1.joints_position.at(0) = 3.83; 
  location_agv1.name = "agv1"; 

  location_agv2.joints_position = home_face_bins.joints_position; 
  // linear actuator at y axis 
  location_agv2.joints_position.at(0) = 0.83; 
  location_agv2.name = "agv2"; 

  location_agv3.joints_position = home_face_bins.joints_position; 
  // linear actuator at y axis 
  location_agv3.joints_position.at(0) = -1.83; 
  location_agv3.name = "agv3"; 

  location_agv4.joints_position = home_face_bins.joints_position; 
  // linear actuator at y axis 
  location_agv4.joints_position.at(0) = -4.33; 
  location_agv4.name = "agv4"; 

  // initialize home position
  this->goToPresetLocation("home_face_belt"); 
  this->goToPresetLocation("home_face_bins"); 
}

void KittingArm::copyCurrentJointsPosition()
{
  // raw pointers are frequently used to refer to the planning group for improved performance.
  // to start, we will create a pointer that references the current robot’s state.
  moveit::core::RobotStatePtr current_state = m_arm_group.getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
          current_state->getJointModelGroup("kitting_arm");

  // next get the current set of joint values for the group.
  current_state->copyJointGroupPositions(joint_model_group, m_joint_group_positions);
}

void KittingArm::print_joints_position() {

  this->copyCurrentJointsPosition(); 
  
  for (auto& joint: m_joint_group_positions) {
    ROS_INFO("%f", joint); 
  }
}

void KittingArm::arm_joint_states_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
{
    if (joint_state_msg->position.size() == 0) {
        ROS_ERROR("[Arm][arm_joint_states_callback_] joint_state_msg->position.size() == 0!");
    }
    m_current_joint_states = *joint_state_msg;
}

void KittingArm::arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
    m_arm_controller_state = *msg;
}


void KittingArm::gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& gripper_state_msg)
{
    m_gripper_state = *gripper_state_msg;
}

void KittingArm::part_task_callback(const ariac_group1::PartTask::ConstPtr& msg)
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 
  // add tasks to task vector
  m_part_task_queue.emplace_back(std::make_unique<nist_gear::Product>(msg->part)); 
  Utility::print_part_pose(msg->part); 
}

nist_gear::VacuumGripperState KittingArm::getGripperState()
{
    return m_gripper_state;
}

void KittingArm::activateGripper()
{
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = true;
    m_gripper_control_client.call(srv);

    ROS_INFO_STREAM("[Arm][activateGripper] DEBUG: srv.response =" << srv.response);
}

void KittingArm::deactivateGripper()
{
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = false;
    m_gripper_control_client.call(srv);

    ROS_INFO_STREAM("[Arm][deactivateGripper] DEBUG: srv.response =" << srv.response);
}

void KittingArm::goToPresetLocation(std::string location_name)
{

    this->copyCurrentJointsPosition(); 

    ArmPresetLocation location;
    if (location_name.compare("home_face_belt") == 0) {
        location = home_face_belt;
    }
    else if (location_name.compare("home_face_bins") == 0) {
        location = home_face_bins;
    }
    else if (location_name.compare("agv1") == 0) {
        location = location_agv1;
    }
    else if (location_name.compare("agv2") == 0) {
        location = location_agv2;
    }
    else if (location_name.compare("agv3") == 0) {
        location = location_agv3;
    }
    else if (location_name.compare("agv4") == 0) {
        location = location_agv4;
    }

    m_joint_group_positions.at(0) = location.joints_position.at(0);
    m_joint_group_positions.at(1) = location.joints_position.at(1);
    m_joint_group_positions.at(2) = location.joints_position.at(2);
    m_joint_group_positions.at(3) = location.joints_position.at(3);
    m_joint_group_positions.at(4) = location.joints_position.at(4);
    m_joint_group_positions.at(5) = location.joints_position.at(5);
    m_joint_group_positions.at(6) = location.joints_position.at(6);

    m_arm_group.setJointValueTarget(m_joint_group_positions);
    this->move_arm_group(); 
}

bool KittingArm::move_arm_group()
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // check a plan is found first then execute the action
    bool success = (m_arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
        m_arm_group.move();

    return success; 
}

void KittingArm::moveBaseTo(double linear_arm_actuator_joint_position) {

    this->copyCurrentJointsPosition(); 
    
    // next, assign a value to only the linear_arm_actuator_joint
    m_joint_group_positions.at(0) = linear_arm_actuator_joint_position;

    // move the arm
    m_arm_group.setJointValueTarget(m_joint_group_positions);
    this->move_arm_group(); 
}

void KittingArm::turnToBins()
{

  this->copyCurrentJointsPosition(); 

  double epsilon = 0.01; 
  if (abs(m_joint_group_positions.at(1) - (-M_PI)) < epsilon) {
    return; 
  }

  m_joint_group_positions.at(1) = -M_PI; 
  m_arm_group.setJointValueTarget(m_joint_group_positions);
  this->move_arm_group(); 
}

void KittingArm::turnToBelt()
{

  this->copyCurrentJointsPosition(); 

  double epsilon = 0.01; 
  if (abs(m_joint_group_positions.at(1)) < epsilon) {
    return; 
  }

  m_joint_group_positions.at(1) = 0; 
  m_arm_group.setJointValueTarget(m_joint_group_positions);
  this->move_arm_group(); 
}


