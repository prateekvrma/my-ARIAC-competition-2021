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
  home_face_belt.joints_pos = default_pos;
  home_face_belt.name = "home_face_belt";

  home_face_bins.joints_pos = default_pos;
  // shoulder_pan_joint
  home_face_bins.joints_pos.at(1) = -M_PI;
  home_face_bins.name = "home_face_bins";

  location_agv1.joints_pos = home_face_bins.joints_pos; 
  // linear actuator at y axis 
  location_agv1.joints_pos.at(0) = 3.83; 
  location_agv1.name = "agv1"; 

  location_agv2.joints_pos = home_face_bins.joints_pos; 
  // linear actuator at y axis 
  location_agv2.joints_pos.at(0) = 0.83; 
  location_agv2.name = "agv2"; 

  location_agv3.joints_pos = home_face_bins.joints_pos; 
  // linear actuator at y axis 
  location_agv3.joints_pos.at(0) = -1.83; 
  location_agv3.name = "agv3"; 

  location_agv4.joints_pos = home_face_bins.joints_pos; 
  // linear actuator at y axis 
  location_agv4.joints_pos.at(0) = -4.33; 
  location_agv4.name = "agv4"; 


  // raw pointers are frequently used to refer to the planning group for improved performance.
  // to start, we will create a pointer that references the current robot’s state.
  moveit::core::RobotStatePtr current_state = m_arm_group.getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
          current_state->getJointModelGroup("kitting_arm");

  // next get the current set of joint values for the group.
  current_state->copyJointGroupPositions(joint_model_group, m_joint_group_positions);
}

void KittingArm::print_joints_position() {
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


