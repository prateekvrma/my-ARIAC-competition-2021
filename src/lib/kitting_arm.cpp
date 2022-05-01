#include "kitting_arm.h"

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/convert.h>
#include <Eigen/Geometry>
#include <math.h>
#include <random>

#include <nist_gear/AGVToAssemblyStation.h>
#include <ariac_group1/IsFaulty.h>
#include <ariac_group1/PartsInCamera.h>
#include <ariac_group1/IsPartPicked.h>
#include <ariac_group1/GetPartPosition.h>
#include <ariac_group1/CheckQualitySensor.h>
#include <ariac_group1/GetCompetitionTime.h>
#include <std_srvs/Trigger.h>
#include <ariac_group1/GetBeltPart.h>
#include <ariac_group1/GetBeltProximitySensor.h>
#include <ariac_group1/GetVacancyPose.h>

using AGVToAssem = nist_gear::AGVToAssemblyStation; 

KittingArm::KittingArm():
  m_nh{"/ariac/kitting"},
  m_planning_group{"/ariac/kitting/robot_description"},
  m_arm_options{"kitting_arm", m_planning_group, m_nh},
  m_arm_group{m_arm_options}, 
  m_shipments{&m_nh}
{
  // publishers to directly control the joints without moveit
  m_arm_joint_trajectory_publisher =
      m_nh.advertise<trajectory_msgs::JointTrajectory>("/ariac/kitting/kitting_arm_controller/command", 10);

  // subscribers
  m_arm_joint_states_subscriber =
      m_nh.subscribe("/ariac/kitting/joint_states", 10, &KittingArm::arm_joint_states_callback, this);
  
  m_arm_controller_state_subscriber =
      m_nh.subscribe("/ariac/kitting/kitting_arm_controller/state", 10, &KittingArm::arm_controller_state_callback, this);

  m_gripper_state_subscriber = 
      m_nh.subscribe("/ariac/kitting/arm/gripper/state", 10, &KittingArm::gripper_state_callback, this);

  // m_part_task_subscriber =
  //     m_nh.subscribe("/part_task", 10, &KittingArm::part_task_callback, this); 

  // clients
  m_gripper_control_client =
      m_nh.serviceClient<nist_gear::VacuumGripperControl>("/ariac/kitting/arm/gripper/control");
  m_gripper_control_client.waitForExistence();

  m_get_parts_client = 
      m_nh.serviceClient<ariac_group1::GetParts>("/sensor_manager/get_parts"); 
  m_get_parts_client.waitForExistence();

  m_is_faulty_client = 
      m_nh.serviceClient<ariac_group1::IsFaulty>("/sensor_manager/is_faulty"); 
  m_is_faulty_client.waitForExistence();

  m_parts_in_camera_client = 
      m_nh.serviceClient<ariac_group1::PartsInCamera>("/sensor_manager/parts_in_camera"); 
  m_parts_in_camera_client.waitForExistence();

  m_is_part_picked_client = 
      m_nh.serviceClient<ariac_group1::IsPartPicked>("/sensor_manager/is_part_picked"); 
  m_is_part_picked_client.waitForExistence();

  m_get_part_position_client = 
      m_nh.serviceClient<ariac_group1::GetPartPosition>("/sensor_manager/get_part_position"); 
  m_get_part_position_client.waitForExistence();

  m_check_quality_sensor_client = 
      m_nh.serviceClient<ariac_group1::CheckQualitySensor>("/sensor_manager/check_quality_sensor"); 
  m_check_quality_sensor_client.waitForExistence();

  m_get_competition_time_client = 
      m_nh.serviceClient<ariac_group1::GetCompetitionTime>("/factory_manager/get_competition_time"); 
  m_get_competition_time_client.waitForExistence();

  m_is_belt_sensor_triggered_client = 
      m_nh.serviceClient<std_srvs::Trigger>("/sensor_manager/is_belt_sensor_triggered"); 
  m_is_belt_sensor_triggered_client.waitForExistence();

  m_get_belt_part_client = 
      m_nh.serviceClient<ariac_group1::GetBeltPart>("/sensor_manager/get_belt_part"); 
  m_get_belt_part_client.waitForExistence();

  m_get_belt_proximity_sensor_client = 
      m_nh.serviceClient<ariac_group1::GetBeltProximitySensor>("/sensor_manager/get_belt_proximity_sensor"); 
  m_get_belt_proximity_sensor_client.waitForExistence();

  m_get_vacancy_pose_client = 
      m_nh.serviceClient<ariac_group1::GetVacancyPose>("/sensor_manager/get_vacancy_pose"); 
  m_get_vacancy_pose_client.waitForExistence();


  for (auto& id: m_agvs_id) {
      m_agvs_dict[id] = std::make_unique<AGV>(&m_nh, id); 
  }

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

  location_belt_part.joints_position = default_pos;
  location_belt_part.joints_position.at(0) = -3;
  location_belt_part.name = "belt_part";

  location_belt_intercept.joints_position = default_pos;
  location_belt_intercept.joints_position.at(1) = -M_PI;
  location_belt_intercept.joints_position.at(2) = -2;
  location_belt_intercept.joints_position.at(3) = -2;
  location_belt_intercept.joints_position.at(5) = 3.05;
  location_belt_intercept.name = "belt_intercept";


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
  location_agv3.joints_position.at(0) = -1.63; 
  location_agv3.name = "agv3"; 

  location_agv4.joints_position = home_face_bins.joints_position; 
  // linear actuator at y axis 
  location_agv4.joints_position.at(0) = -4.1; 
  location_agv4.name = "agv4"; 

  location_bins0.joints_position = home_face_bins.joints_position; 
  // linear actuator at y axis 
  location_bins0.joints_position.at(0) = 3; 
  location_bins0.name = "bins0"; 

  location_bins1.joints_position = home_face_bins.joints_position; 
  // linear actuator at y axis 
  location_bins1.joints_position.at(0) = -4; 
  location_bins1.name = "bins1"; 

  m_arm_group.setPlanningTime(10.0); 


  // initialize home position
  this->goToPresetLocation("home_face_belt"); 
  this->goToPresetLocation("home_face_bins"); 
}

void KittingArm::copyCurrentJointsPosition()
{
  moveit::core::RobotStatePtr current_state = m_arm_group.getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
          current_state->getJointModelGroup("kitting_arm");

  // next get the current set of joint values for the group.
  current_state->copyJointGroupPositions(joint_model_group, m_joint_group_positions);
}

void KittingArm::print_joint_group_positions() {
  ROS_INFO("Target Joint positions: "); 
  for (int i; i < m_joint_group_positions.size(); ++i) {
    ROS_INFO("  Joint %d: %f", i, m_joint_group_positions.at(i)); 
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

// void KittingArm::part_task_callback(const ariac_group1::PartTask::ConstPtr& msg)
// {
//   const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 
//   // add tasks to task vector
//   m_part_task_queue.emplace_back(std::make_tuple(msg->priority * Constants::PriorityWeight::Ratio::HIGH_PRIORITY,
//                                                  std::make_unique<ariac_group1::PartTask>(*msg))); 
//   if (not m_shipments_total_parts.count(msg->shipment_type)) {
//     m_shipments_total_parts[msg->shipment_type] = msg->total_parts; 
//     ROS_INFO("Receive shipment %s including %d parts", msg->shipment_type.c_str(), msg->total_parts); 
//   }
// }

// void KittingArm::print_shipments_total_parts() {
//   ROS_INFO("Shipment total parts: "); 
//   for (auto& part_count: m_shipments.shipment_record) {
//     ROS_INFO("  %s: %d", part_count.first.c_str(), part_count.second); 
//   }
//
// }

nist_gear::VacuumGripperState KittingArm::getGripperState()
{
    return m_gripper_state;
}

void KittingArm::activateGripper()
{
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = true;
    m_gripper_control_client.call(srv);

    ROS_INFO_STREAM("Activate gripper " << srv.response);
}

void KittingArm::deactivateGripper()
{
    nist_gear::VacuumGripperControl srv;
    srv.request.enable = false;
    m_gripper_control_client.call(srv);

    ROS_INFO_STREAM("Deactivate gripper " << srv.response);
}

void KittingArm::goToPresetLocation(std::string location_name)
{

    this->copyCurrentJointsPosition(); 

    ArmPresetLocation location;
    if (location_name.compare("home_face_belt") == 0) {
        ROS_INFO("Move to home_face_belt"); 
        location = home_face_belt;
    }
    else if (location_name.compare("home_face_bins") == 0) {
        ROS_INFO("Move to home_face_bins"); 
        location = home_face_bins;
    }
    else if (location_name.compare("agv1") == 0 or
             location_name.find("ks1") != std::string::npos) {
        ROS_INFO("Move to agv1"); 
        location = location_agv1;
    }
    else if (location_name.compare("agv2") == 0 or 
             location_name.find("ks2") != std::string::npos) {
        ROS_INFO("Move to agv2"); 
        location = location_agv2;
    }
    else if (location_name.compare("agv3") == 0 or
             location_name.find("ks3") != std::string::npos) {
        ROS_INFO("Move to agv3"); 
        location = location_agv3;
    }
    else if (location_name.compare("agv4") == 0 or 
             location_name.find("ks4") != std::string::npos) {
        ROS_INFO("Move to agv4"); 
        location = location_agv4;
    }
    else if (location_name.find("bins0") != std::string::npos) {
        ROS_INFO("Move to bins0"); 
        location = location_bins0;
    }
    else if (location_name.find("bins1") != std::string::npos) {
        ROS_INFO("Move to bins1"); 
        location = location_bins1;

    }
    else if (location_name.find("belt_part") != std::string::npos) {
        ROS_INFO("Move to belt part"); 
        location = location_belt_part;
    }
    else if (location_name.find("belt_intercept") != std::string::npos) {
        ROS_INFO("Move to belt intercept"); 
        location = location_belt_intercept;
    }

    m_joint_group_positions.at(0) = location.joints_position.at(0);
    m_joint_group_positions.at(1) = location.joints_position.at(1);
    m_joint_group_positions.at(2) = location.joints_position.at(2);
    m_joint_group_positions.at(3) = location.joints_position.at(3);
    m_joint_group_positions.at(4) = location.joints_position.at(4);
    m_joint_group_positions.at(5) = location.joints_position.at(5);
    m_joint_group_positions.at(6) = location.joints_position.at(6);

    m_arm_group.setMaxVelocityScalingFactor(1.0);
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
    else
        ROS_INFO("Path planning fails"); 

    return success; 
}

void KittingArm::moveBaseTo(double linear_arm_actuator_joint_position)
{

    this->copyCurrentJointsPosition(); 
    
    // next, assign a value to only the linear_arm_actuator_joint
    m_joint_group_positions.at(0) = linear_arm_actuator_joint_position;

    // move the arm
    m_arm_group.setJointValueTarget(m_joint_group_positions);
    this->move_arm_group(); 
}

void KittingArm::resetArm()
{
    this->copyCurrentJointsPosition(); 
    m_joint_group_positions.at(1) = -M_PI;
    m_joint_group_positions.at(2) = -1.25;
    m_joint_group_positions.at(3) = 1.74;
    m_joint_group_positions.at(4) = -2.04;
    m_joint_group_positions.at(5) = -1.57;
    m_joint_group_positions.at(6) = 0;
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

void KittingArm::lift()
{
  this->copyCurrentJointsPosition(); 
  m_joint_group_positions.at(2) -= 0.3; 
  m_joint_group_positions.at(3) -= 0.1; 
  m_arm_group.setJointValueTarget(m_joint_group_positions);
  this->move_arm_group(); 
}

void KittingArm::setPickConstraints()
{
  // set constraint to shoulder lift and elbow
  moveit_msgs::Constraints constraints;

  moveit_msgs::JointConstraint joint_constraint; 
  // joint_constraint.joint_name = "shoulder_pan_joint"; 
  // joint_constraint.position = -M_PI;  
  // joint_constraint.tolerance_above = 1.2; 
  // joint_constraint.tolerance_below = 1.2; 
  // joint_constraint.weight = 1; 
  //
  // constraints.joint_constraints.push_back(joint_constraint); 
  //

  joint_constraint.joint_name = "shoulder_lift_joint"; 
  joint_constraint.position = -1.25;  
  joint_constraint.tolerance_above = 1.05; 
  joint_constraint.tolerance_below = 0.5; 
  joint_constraint.weight = 0.8; 

  constraints.joint_constraints.push_back(joint_constraint); 

  joint_constraint.joint_name = "elbow_joint"; 
  joint_constraint.position = 1.74;  
  joint_constraint.tolerance_above = 0.76; 
  joint_constraint.tolerance_below = 1.7; 
  joint_constraint.weight = 0.8; 

  constraints.joint_constraints.push_back(joint_constraint); 

  joint_constraint.joint_name = "wrist_1_joint"; 
  joint_constraint.position = -2.04;  
  joint_constraint.tolerance_above = 1.4; 
  joint_constraint.tolerance_below = 1.4; 
  joint_constraint.weight = 0.8; 

  constraints.joint_constraints.push_back(joint_constraint); 

  joint_constraint.joint_name = "wrist_2_joint"; 
  joint_constraint.position = -1.57;  
  joint_constraint.tolerance_above = 0.1; 
  joint_constraint.tolerance_below = 0.1; 
  joint_constraint.weight = 1; 

  constraints.joint_constraints.push_back(joint_constraint); 


  m_arm_group.setPathConstraints(constraints);

}

bool KittingArm::moveTargetPose(const geometry_msgs::Pose& pose)
{
  this->setPickConstraints(); 

  unsigned int max_attempts = 1; 
  int attempts = 0; 
  std::vector<double> current_joint_group_positions = m_joint_group_positions; 
  std::random_device rd; 
  std::mt19937 gen(rd()); 

  while(attempts < max_attempts) {
      attempts++; 

      if (attempts > 1) {
      // add random motion to try again
          std::normal_distribution<double> gauss_dist(-0.05, 0.05); 
          auto random_dist = gauss_dist(gen); 
          m_joint_group_positions.at(0) += random_dist; 
          this->moveBaseTo(m_joint_group_positions.at(0)); 
      }

      ROS_INFO("Set pose attempts: %d", attempts); 
      m_joint_group_positions = current_joint_group_positions; 
      moveit::core::RobotStatePtr current_state = m_arm_group.getCurrentState();
      const moveit::core::JointModelGroup* joint_model_group =
              current_state->getJointModelGroup("kitting_arm");

      double timeout = 5; 
      bool found_ik = current_state->setFromIK(joint_model_group, pose, timeout); 

      if (found_ik) {
        std::vector<double> target_joint_group_positions; 
        current_state->copyJointGroupPositions(joint_model_group, target_joint_group_positions);
        bool target_joints_infeasible = false; 

        auto joint_1 = target_joint_group_positions.at(1);  
        auto joint_2 = target_joint_group_positions.at(2);  
        auto joint_3 = target_joint_group_positions.at(3);  
        auto joint_4 = target_joint_group_positions.at(4);  
        // if (joint_1 > 0) {
        //     joint_1 = joint_1 - M_PI * 2; 
        // }
        //
        if (joint_2 > 0) {
            joint_2 = joint_2 - M_PI * 2; 
        }

        if (joint_3 < 0) {
            joint_3 = joint_3 + M_PI * 2; 
        }

        if (joint_4 > 0) {
            joint_4 = joint_4 - M_PI * 2; 
        }

        // if (joint_1 > -1.94 or joint_1 < -4.34) {
        //   ROS_INFO("Target joint 1 infisible: %f", target_joint_group_positions.at(2)); 
        //   target_joints_infeasible = true; 
        // }
        //
        if (joint_2 > -0.17 or joint_2 < -1.74) { 
          ROS_INFO("Target joint 2 infisible: %f", target_joint_group_positions.at(2)); 
          target_joints_infeasible = true; 
        }

        if (joint_3 > 2.5 or joint_3 < 0.05) { 
          ROS_INFO("Target joint 3 infisible: %f", target_joint_group_positions.at(3)); 
          target_joints_infeasible = true; 
        }

        if (joint_4 > -0.7 or joint_4 < -3.45) { 
          ROS_INFO("Target joint 4 infisible: %f", target_joint_group_positions.at(4)); 
          target_joints_infeasible = true; 
        }

        if (target_joints_infeasible) {

          ros::Duration(3.0).sleep(); 
          m_arm_group.setMaxVelocityScalingFactor(0.5);
          m_joint_group_positions = current_joint_group_positions; 
          m_arm_group.setJointValueTarget(m_joint_group_positions);
          this->move_arm_group(); 
          m_arm_group.setMaxVelocityScalingFactor(1.0);
          continue; 
        }

        m_joint_group_positions.at(0) = target_joint_group_positions.at(0); 
        m_joint_group_positions.at(1) = target_joint_group_positions.at(1); 
        m_arm_group.setJointValueTarget(m_joint_group_positions);
        this->move_arm_group(); 

        m_joint_group_positions = target_joint_group_positions; 
        m_arm_group.setJointValueTarget(m_joint_group_positions);
        this->move_arm_group(); 
        m_arm_group.clearPathConstraints();
        return true; 
      }
  }

  this->copyCurrentJointsPosition(); 
  m_arm_group.clearPathConstraints();
  return false; 

}

bool KittingArm::pickPart(std::string part_type, 
                          const geometry_msgs::Pose& part_init_pose,
                          std::string camera_id) 
{
    ROS_INFO("---Start pick part"); 
    m_arm_group.setMaxVelocityScalingFactor(1.0);

    geometry_msgs::Pose arm_ee_link_pose = m_arm_group.getCurrentPose().pose;
    auto flat_orientation = Utility::motioncontrol::quaternionFromEuler(0, 1.57, 0);
    arm_ee_link_pose.orientation.x = flat_orientation.getX();
    arm_ee_link_pose.orientation.y = flat_orientation.getY();
    arm_ee_link_pose.orientation.z = flat_orientation.getZ();
    arm_ee_link_pose.orientation.w = flat_orientation.getW();
    
    // preset z depending on the part type
    double z_pos{};
    if (part_type.find("pump") != std::string::npos) {
        z_pos = 0.075;
        if (camera_id.find("ks") != std::string::npos) {
          z_pos -= 0.0052;
        }
    }
    if (part_type.find("sensor") != std::string::npos) {
        z_pos = 0.05;
        if (camera_id.find("ks") != std::string::npos) {
          z_pos -= 0.005;
        }
    }
    if (part_type.find("battery") != std::string::npos) {
        z_pos = 0.052;
        if (camera_id.find("ks") != std::string::npos) {
          z_pos -= 0.005;
        }
    }
    if (part_type.find("regulator") != std::string::npos) {
        z_pos = 0.057;
        if (camera_id.find("ks") != std::string::npos) {
          z_pos -= 0.005;
        }
    }

    

    // post-grasp pose 
    // store the pose of the arm before it goes down to pick the part
    // we will bring the arm back to this pose after picking up the part
    auto postgrasp_pose = part_init_pose;
    postgrasp_pose.orientation = arm_ee_link_pose.orientation;
    postgrasp_pose.position.z = part_init_pose.position.z + z_pos + 0.05;

    if (this->check_emergency_interrupt()) {
        return false; 
    }

    // m_arm_group.setPoseTarget(postgrasp_pose);
    // m_arm_group.move();
    ROS_INFO("Move to postgrasp pose"); 
    if (not this->moveTargetPose(postgrasp_pose)) {
      ROS_INFO("---End pick part: IK not found for postgrasp"); 
      return false; 
    }

    if (this->check_emergency_interrupt()) {
        return false; 
    }

    // pregrasp pose: right above the part
    auto pregrasp_pose = part_init_pose;
    pregrasp_pose.orientation = arm_ee_link_pose.orientation;
    pregrasp_pose.position.z = part_init_pose.position.z + z_pos;

    // activate gripper
    // sometimes it does not activate right away
    // so we are doing this in a loop
    while (!m_gripper_state.enabled) {
        activateGripper();
    }

    ROS_INFO("Move to pregrasp pose"); 
    if (not this->moveTargetPose(pregrasp_pose)) {
      ROS_INFO("IK not found for grasp"); 
      deactivateGripper();
      this->lift(); 
      return false; 
    }

    if (this->check_emergency_interrupt()) {
      return false; 
    }

    ros::Duration(0.5).sleep();
    
    m_arm_group.setMaxVelocityScalingFactor(0.5);
    auto grasp_pose = pregrasp_pose; 
    // this->setPickConstraints(); 
    ROS_INFO("Start grasping"); 
    // move the arm 1 mm down until the part is attached
    int count = 0; 
    int trial = 0; 
    double step = 0.001;
    while (!m_gripper_state.attached) {
        grasp_pose.position.z -= step;
        m_arm_group.setPoseTarget(grasp_pose);
        m_arm_group.move();
        count++; 
        ros::Duration(0.3).sleep();
        // if (step > 0.0008) {
          // step -= 0.0001; 
        // }
        geometry_msgs::Pose arm_ee_link_pose = m_arm_group.getCurrentPose().pose;
        if (arm_ee_link_pose.position.z < 0.76) {
          ROS_INFO("---End pick part: Arm moving lower then part, abort"); 
          this->resetArm(); 
          deactivateGripper();
          return false; 
        }

        if (trial > 3) {
          ROS_INFO("Hard to grasp. Abort"); 
          this->resetArm(); 
          deactivateGripper();
          return false; 
        }

        if (count > 2) {
          if (this->check_emergency_interrupt()) {
              return false; 
          }
        }

        if (count > 7) {
          this->lift(); 
          ROS_INFO("Hard to grasp. Move back to pregrasp pose"); 
          if (not this->moveTargetPose(pregrasp_pose)) {
            ROS_INFO("IK not found for grasp"); 
            deactivateGripper();
            this->lift(); 
            return false; 
          }

          grasp_pose = pregrasp_pose; 
          count = 0; 
          trial++; 
        }
    }
    // m_arm_group.clearPathConstraints();
    
    m_arm_group.setMaxVelocityScalingFactor(1.0);
    m_arm_group.setMaxAccelerationScalingFactor(1.0);
    ROS_INFO_STREAM("[Gripper] = object attached");
    ros::Duration(0.5).sleep();

    ROS_INFO("Move back to postgrasp pose"); 
    if (not this->moveTargetPose(postgrasp_pose)) {
      ROS_INFO("---End pick part: IK not found for postgrasp"); 
      this->lift(); 
      return false; 
    }
    // m_arm_group.setPoseTarget(postgrasp_pose);
    // m_arm_group.move();
    ros::Duration(0.5).sleep();

    this->lift(); 

    ariac_group1::IsPartPicked srv; 
    srv.request.camera_id = camera_id; 
    srv.request.part.type = part_type; 
    srv.request.part.pose = part_init_pose; 
    if (m_is_part_picked_client.call(srv)) {
      if (srv.response.picked) {
        ROS_INFO("---End pick part: pick success"); 
        return true; 
      }
      else {
        ROS_INFO("---End pick part: pick fails"); 
        deactivateGripper();
        return false; 
      }
    } 
    else {
      ROS_INFO("---End pick part: no such camera: %s", camera_id.c_str()); 
      deactivateGripper();
      return false; 
    }
}

geometry_msgs::Pose KittingArm::placePart(std::string part_type, 
                                          geometry_msgs::Pose part_init_pose, 
                                          geometry_msgs::Pose part_pose_in_frame, 
                                          std::string agv)
{
    goToPresetLocation(agv);

    // get the target pose of the part in the world frame
    auto target_pose_in_world = Utility::motioncontrol::transformToWorldFrame(
         part_pose_in_frame,
         agv);

    ROS_INFO("---Start place part"); 
    Utility::print_pose(target_pose_in_world); 

   

    geometry_msgs::Pose arm_ee_link_pose = m_arm_group.getCurrentPose().pose;
    auto flat_orientation = Utility::motioncontrol::quaternionFromEuler(0, 1.57, 0);
    arm_ee_link_pose = m_arm_group.getCurrentPose().pose;
    arm_ee_link_pose.orientation.x = flat_orientation.getX();
    arm_ee_link_pose.orientation.y = flat_orientation.getY();
    arm_ee_link_pose.orientation.z = flat_orientation.getZ();
    arm_ee_link_pose.orientation.w = flat_orientation.getW();

    // store the current orientation of the end effector now
    // so we can reuse it later
    tf2::Quaternion q_current(
        arm_ee_link_pose.orientation.x,
        arm_ee_link_pose.orientation.y,
        arm_ee_link_pose.orientation.z,
        arm_ee_link_pose.orientation.w);
    
    // move the arm above the agv
    // gripper stays at the current z
    // only modify its x and y based on the part to grasp
    // In this case we do not need to use preset locations
    // everything is done dynamically
    arm_ee_link_pose.position.x = target_pose_in_world.position.x;
    arm_ee_link_pose.position.y = target_pose_in_world.position.y;
    // move the arm
    m_arm_group.setMaxVelocityScalingFactor(0.8);

    ROS_INFO("Move to place pose above"); 
    while (not this->moveTargetPose(arm_ee_link_pose)) {
      ROS_INFO("IK not found for place pose above"); 
      goToPresetLocation(agv);

      std::random_device rd; 
      std::mt19937 gen(rd());
      std::normal_distribution<double> gauss_dist(-0.1, 0.1); 
      auto random_dist = gauss_dist(gen); 
      m_joint_group_positions.at(0) += random_dist; 
      this->moveBaseTo(m_joint_group_positions.at(0));
      ros::Duration(3.0).sleep();
    }

    // m_arm_group.setPoseTarget(arm_ee_link_pose);
    // m_arm_group.move();

  
    // orientation of the part in the bin, in world frame
    tf2::Quaternion q_init_part(
        part_init_pose.orientation.x,
        part_init_pose.orientation.y,
        part_init_pose.orientation.z,
        part_init_pose.orientation.w);

    // orientation of the part in the tray, in world frame
    tf2::Quaternion q_target_part(
        target_pose_in_world.orientation.x,
        target_pose_in_world.orientation.y,
        target_pose_in_world.orientation.z,
        target_pose_in_world.orientation.w);

    // relative rotation between init and target
    tf2::Quaternion q_rot = q_target_part * q_init_part.inverse();

    // apply this rotation to the current gripper rotation
    tf2::Quaternion q_rslt = q_rot * q_current;
    q_rslt.normalize();

    double z_margin{};
    if (part_type.find("pump") != std::string::npos) {
        z_margin = 0.14;
    }
    else {
      z_margin = 0.11; 
    }

    // else if (part_type.find("regulator") != std::string::npos) {
      // z_margin = 0.11; 
    // }
    //
    // orientation of the gripper when placing the part in the tray
    target_pose_in_world.orientation.x = q_rslt.x();
    target_pose_in_world.orientation.y = q_rslt.y();
    target_pose_in_world.orientation.z = q_rslt.z();
    target_pose_in_world.orientation.w = q_rslt.w();
    target_pose_in_world.position.z += z_margin;

    // auto target_rpy = Utility::motioncontrol::eulerFromQuaternion(target_pose_in_world);
    // auto q_target_pose_flat = Utility::motioncontrol::quaternionFromEuler(target_rpy.at(0), target_rpy.at(1), target_rpy.at(2));

    // target_pose_in_world.orientation.x = q_target_pose_flat.x();
    // target_pose_in_world.orientation.y = q_target_pose_flat.y();
    // target_pose_in_world.orientation.z = q_target_pose_flat.z();
    // target_pose_in_world.orientation.w = q_target_pose_flat.w();

    // m_arm_group.setMaxVelocityScalingFactor(0.1);
    // m_arm_group.setPoseTarget(target_pose_in_world);
    // m_arm_group.move();
    ROS_INFO("Move to place pose"); 
    while (not this->moveTargetPose(target_pose_in_world)) {
      ROS_INFO("IK not found for place"); 
      goToPresetLocation(agv);

      std::random_device rd; 
      std::mt19937 gen(rd());
      std::normal_distribution<double> gauss_dist(-0.05, 0.05); 
      auto random_dist = gauss_dist(gen); 
      m_joint_group_positions.at(0) += random_dist; 
      this->moveBaseTo(m_joint_group_positions.at(0));
      ros::Duration(0.1).sleep();
    }
    
    ros::Duration(1.0).sleep();
    deactivateGripper();

    m_arm_group.setMaxVelocityScalingFactor(1.0);
    this->lift(); 
    ROS_INFO("---End place part"); 

    return target_pose_in_world;
}
bool KittingArm::discard_faulty(const nist_gear::Model& faulty_part, std::string target_agv)
{
      std::string camera_id; 
      if (target_agv.compare("agv1") == 0) {
        camera_id = "logical_camera_ks1"; 
      }
      else if (target_agv.compare("agv2") == 0) {
        camera_id = "logical_camera_ks2"; 
      }
      else if (target_agv.compare("agv3") == 0) {
        camera_id = "logical_camera_ks3"; 
      }
      else if (target_agv.compare("agv4") == 0) {
        camera_id = "logical_camera_ks4"; 
      } 

      ROS_INFO("---Start discard part"); 
      ariac_group1::GetPartPosition srv; 
      srv.request.camera_id = camera_id; 
      srv.request.part = faulty_part; 
      geometry_msgs::Pose faulty_part_pose; 
      if (m_get_part_position_client.call(srv)) {
        faulty_part_pose = srv.response.pose; 
      } else {
        ROS_INFO("No rectified pose"); 
      }
      this->copyCurrentJointsPosition(); 

      std::random_device rd; 
      std::mt19937 gen(rd());

      int trial_count = 1; 
      ROS_INFO("  trial %d", trial_count); 
      while (not pickPart(faulty_part.type, faulty_part_pose, camera_id)) {
        
        ROS_INFO("Discard picking fails, try new joint config for discarding"); 
        ros::Duration(3.0).sleep(); 
        this->resetArm(); 
        trial_count++; 
        ROS_INFO("trial %d", trial_count); 

        if (m_joint_group_positions.at(0) > 4 or 
            m_joint_group_positions.at(0) < -4 ) {
          return false; 
        }

        if (trial_count == 1) {
          std::normal_distribution<double> gauss_dist(-0.05, 0.05); 
          auto random_dist = gauss_dist(gen); 
          m_joint_group_positions.at(0) += random_dist; 
          m_arm_group.setJointValueTarget(m_joint_group_positions);
          this->move_arm_group(); 
        }

        if (trial_count > 1) {
          std::normal_distribution<double> gauss_dist(-0.3, 0.3); 
          auto random_dist = gauss_dist(gen); 
          moveBaseTo(m_joint_group_positions.at(0) + random_dist);
        }

        this->copyCurrentJointsPosition(); 
        
        if (trial_count > 5) return false; 
      }
      ROS_INFO("Discard picking success"); 

      goToPresetLocation("home_face_bins");
      ros::Duration(0.5).sleep();
      deactivateGripper();
      ROS_INFO("---End discard part"); 
      return true; 
}

bool KittingArm::check_faulty(const nist_gear::Model& faulty_part)
{
  ariac_group1::IsFaulty srv; 
  
  srv.request.part = faulty_part; 
  Utility::print_part_pose(faulty_part); 
  m_is_faulty_client.call(srv); 

  return srv.response.faulty; 
}

bool KittingArm::movePart(const ariac_group1::PartInfo& part_init_info, const ariac_group1::PartTask& part_task) {
    auto part_init_pose_in_world = part_init_info.part.pose;
    auto camera_id = part_init_info.camera_id; 
    auto part_type = part_init_info.part.type; 
    auto target_pose_in_frame = part_task.part.pose; 
    auto target_agv = part_task.agv_id; 

    if (this->check_emergency_interrupt()) {
        return false; 
    }

    // use -0.3 to reduce awkward pick trajectory
    // goToPresetLocation(camera_id);
    moveBaseTo(part_init_pose_in_world.position.y - 0.3);

    if (this->check_emergency_interrupt()) {
        return false; 
    }

    if (pickPart(part_type, part_init_pose_in_world, camera_id)) {
        auto target_pose_in_world = placePart(part_type, part_init_pose_in_world, target_pose_in_frame, target_agv);
        ros::Duration(1).sleep(); 

        nist_gear::Model faulty_part; 
        faulty_part.type = part_type; 
        faulty_part.pose = target_pose_in_world; 

        ROS_INFO("Check faulty"); 
        if (this->check_faulty(faulty_part)) {
          if (this->check_emergency_interrupt()) {
              // do emergency first, discard later
              return true; 
          }

          ROS_INFO("Found faulty part:"); 
          Utility::print_part_pose(faulty_part); 
          bool discard_success = this->discard_faulty(faulty_part, target_agv); 
          this->check_emergency_interrupt(); 

          if (discard_success) {
            // if discard success, move part fails
            return false; 
          }
          else {
            // if discard fails, assume move part success
            // discard later
            return true; 
          }
        }
        ROS_INFO("Part not faulty"); 
        return true; 
    }
    else {
      return false; 
    }
}

bool KittingArm::check_emergency_interrupt()
{
  ROS_INFO("Checking for emergency interrupt"); 
  ariac_group1::GetBeltProximitySensor belt_proximity_sensor_srv; 

  bool interrupt = false; 

  bool get_belt_part_available = false; 

  int count = 0; 

  do {
    m_get_belt_proximity_sensor_client.call(belt_proximity_sensor_srv); 
    auto range = belt_proximity_sensor_srv.response.range; 

    ariac_group1::GetVacancyPose vacancy_pose_srv; 
    m_get_vacancy_pose_client.call(vacancy_pose_srv); 
    auto& vacancy_poses = vacancy_pose_srv.response.vacancy_poses;  

    get_belt_part_available = range > 0 and not vacancy_poses.empty(); 
    if (get_belt_part_available) {
      ROS_INFO("Belt sensor triggered"); 
      deactivateGripper();
      ros::Duration(0.1).sleep(); 
      this->resetArm(); 
      if (this->get_belt_part(range)) {
          this->place_to_vacancy(vacancy_poses.at(0)); 
      }
      this->resetArm();
      interrupt = true; 
    }
    count++; 
  }
  while (get_belt_part_available or count < 3);  

  if (m_shipments.is_high_priority_alert()) {
      interrupt = true; 
  }
     
  return interrupt; 
}

void KittingArm::move_to_belt_intercept_pose(const geometry_msgs::Pose& belt_part)
{
    this->goToPresetLocation("belt_intercept"); 
    geometry_msgs::Pose arm_ee_link_pose = m_arm_group.getCurrentPose().pose;
    auto flat_orientation = Utility::motioncontrol::quaternionFromEuler(2, 0, 1.57);
    arm_ee_link_pose.orientation.x = flat_orientation.getX();
    arm_ee_link_pose.orientation.y = flat_orientation.getY();
    arm_ee_link_pose.orientation.z = flat_orientation.getZ();
    arm_ee_link_pose.orientation.w = flat_orientation.getW();

    arm_ee_link_pose.position.x = belt_part.position.x; 
    arm_ee_link_pose.position.z = belt_part.position.z;

    m_arm_group.setPoseTarget(arm_ee_link_pose); 
    m_arm_group.move(); 
}

bool KittingArm::get_belt_part(double range)
{
    // ariac_group1::GetBeltPart srv; 
    // do {
    //     m_get_belt_part_client.call(srv); 
    //     ros::Duration(0.2).sleep(); 
    //
    // } while(srv.response.part.type.empty()); 
    //
    ROS_INFO("Get belt part"); 
    // Utility::print_part_pose(srv.response.part); 
    //

    // geometry_msgs::Pose arm_ee_link_pose = m_arm_group.getCurrentPose().pose;
    // auto flat_orientation = Utility::motioncontrol::quaternionFromEuler(0, 1.57, 0);
    // arm_ee_link_pose.orientation.x = flat_orientation.getX();
    // arm_ee_link_pose.orientation.y = flat_orientation.getY();
    // arm_ee_link_pose.orientation.z = flat_orientation.getZ();
    // arm_ee_link_pose.orientation.w = flat_orientation.getW();
    //
    // // preset z depending on the part type
    // auto part_type = srv.response.part.type; 
    // double z_pos{};
    // if (part_type.find("pump") != std::string::npos) {
    //     z_pos = 0.07;
    // }
    // if (part_type.find("sensor") != std::string::npos) {
    //     z_pos = 0.05;
    // }
    // if (part_type.find("battery") != std::string::npos) {
    //     z_pos = 0.052;
    // }
    // if (part_type.find("regulator") != std::string::npos) {
    //     z_pos = 0.057;
    // }

    // pregrasp pose: right above the part
    // auto pregrasp_pose = srv.response.part.pose;
    // pregrasp_pose.orientation = arm_ee_link_pose.orientation;
    // pregrasp_pose.position.y = srv.response.part.pose.position.y - 0.5;
    // pregrasp_pose.position.z = srv.response.part.pose.position.z + z_pos;
    //
    // activate gripper
    // sometimes it does not activate right away
    // so we are doing this in a loop
    while (!m_gripper_state.enabled) {
        activateGripper();
    }

    geometry_msgs::Pose belt_part; 
    belt_part.position.x = -0.65 + range; 
    // belt_part.position.z = 0.93; 
    belt_part.position.z = 0.945; 

    this->move_to_belt_intercept_pose(belt_part); 

    m_arm_group.setMaxVelocityScalingFactor(0.6);
    int count = 0; 
    while (!m_gripper_state.attached) {
        ROS_INFO("Not attached"); 
        moveBaseTo(m_current_joint_states.position.at(1) + 0.3); 
        ros::Duration(0.2).sleep(); 
        count++; 
        if (m_current_joint_states.position.at(1) > 3) {
            ROS_INFO("No belt part"); 
            deactivateGripper();
            return false; 
        }
    }

    ROS_INFO("Attached!!!"); 

    m_arm_group.setMaxVelocityScalingFactor(1.0);
    this->resetArm(); 
    ros::Duration(0.1).sleep(); 
    return true; 

}

void KittingArm::place_to_vacancy(const geometry_msgs::Pose& vacancy_pose)
{
    ROS_INFO("Place to vacancy"); 
    geometry_msgs::Pose arm_ee_link_pose = m_arm_group.getCurrentPose().pose;
    auto flat_orientation = Utility::motioncontrol::quaternionFromEuler(2, 0, 1.57);
    arm_ee_link_pose.orientation.x = flat_orientation.getX();
    arm_ee_link_pose.orientation.y = flat_orientation.getY();
    arm_ee_link_pose.orientation.z = flat_orientation.getZ();
    arm_ee_link_pose.orientation.w = flat_orientation.getW();

    arm_ee_link_pose.position.x = vacancy_pose.position.x; 
    arm_ee_link_pose.position.y = vacancy_pose.position.y - 0.05; 
    arm_ee_link_pose.position.z = vacancy_pose.position.z + 0.2;

    m_arm_group.setPoseTarget(arm_ee_link_pose); 
    m_arm_group.move();
    ros::Duration(0.1).sleep(); 
    deactivateGripper();
    ros::Duration(0.1).sleep(); 
    this->lift(); 
}

bool KittingArm::get_order()
{
  ros::Duration(1.0).sleep();
  ros::Rate wait_rate(20); 
  this->check_emergency_interrupt();  
  // check if there are tasks and make sure ROS is running 
  while (ros::ok()) {
    ROS_INFO_THROTTLE(3, "Waiting for part task.");
    this->check_emergency_interrupt();  

    m_shipments.update_part_task_queue(m_part_task_queue); 

    if (not m_part_task_queue.empty()) {
        break; 
    }
    
    wait_rate.sleep(); 
  }

  ROS_INFO("Received part task"); 
  return true; 
}

void KittingArm::plan()
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 
  std::sort(m_part_task_queue.begin(), m_part_task_queue.end()); 
}

void KittingArm::execute()
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 

  ROS_INFO("==============================================="); 
  ROS_INFO("Task queue size: %d", (int)m_part_task_queue.size()); 
  auto& part_task_info = m_part_task_queue.back(); 
  auto& priority = std::get<0>(part_task_info); 
  auto& part_task = *std::get<1>(part_task_info); 

  auto shipment_state = this->check_shipment_state(part_task); 
  m_shipments.shipments_record[part_task.shipment_type]->state = shipment_state; 
  this->process_shipment_state(shipment_state, part_task, priority); 
  if (shipment_state != ShipmentState::NOT_READY) {
    return; 
  }

  ROS_INFO("Shipment: %s", part_task.shipment_type.c_str()); 
  ROS_INFO("Priority: %d", priority); 
  ROS_INFO("AGV: %s", part_task.agv_id.c_str()); 
  ROS_INFO("Station id: %s", part_task.station_id.c_str()); 
  ROS_INFO("Part type: %s", part_task.part.type.c_str()); 
  // ros::Duration(30.0).sleep();
  //
  ariac_group1::GetParts get_parts_srv; 
  get_parts_srv.request.type = part_task.part.type; 

  m_get_parts_client.call(get_parts_srv); 

  auto& parts_info = get_parts_srv.response.parts_info; 

  if (not parts_info.empty()) {
    auto part_init_info = parts_info[0]; 
    int idx = 0; 

    // don't use part on ks unless necessary
    while (part_init_info.camera_id.find("ks") != std::string::npos) {
      if (idx >= (parts_info.size() - 1)) {
        if (priority >= Constants::PriorityWeight::Level::HIGH) {
          ROS_INFO("High priority gets part from agv"); 
          break; 
        }
        ROS_INFO("No enough part for %s", part_task.part.type.c_str()); 
        priority += Constants::PriorityWeight::Penalty::NO_PART;  
        if (this->check_insufficient_shipment(priority)) {
          ROS_INFO("Part task %s %s infeasible, discard part task", part_task.shipment_type.c_str(), part_task.part.type.c_str()); 
          this->clear_part_task(part_task, priority); 
        }
        return; 
      }
      idx++; 
      part_init_info = parts_info[idx]; 
    }

    ROS_INFO("Found %s upder %s", part_task.part.type.c_str(), part_init_info.camera_id.c_str()); 
    ROS_INFO("Part location: "); 
    Utility::print_part_pose(part_init_info.part); 

    bool success = this->movePart(part_init_info, part_task); 
    if (success) {
      ROS_INFO("Move part success"); 
      this->clear_part_task(part_task, priority); 
      return; 

    }
    else {
      ROS_INFO("Move part fails"); 
      priority += Constants::PriorityWeight::Penalty::MOVE_FAILS; 
      return; 
    }
  }
  else {
    ROS_INFO("No valid %s, back to task queue", part_task.part.type.c_str()); 
    priority += Constants::PriorityWeight::Penalty::NO_PART;  
    ROS_INFO("Priority decrease to %d", priority); 
    
    if (this->check_insufficient_shipment(priority)) {
      ROS_INFO("Part task %s %s infeasible, discard part task", part_task.shipment_type.c_str(), part_task.part.type.c_str()); 
      this->clear_part_task(part_task, priority); 
    }
     
    return; 
  }

}

bool KittingArm::check_insufficient_shipment(int priority)
{
  if (priority < -6) {
    ariac_group1::GetCompetitionTime get_comp_time_srv; 
    m_get_competition_time_client.call(get_comp_time_srv); 
    auto competition_time = get_comp_time_srv.response.competition_time;
    if (competition_time > 25) {
      return true; 
    }
  }
  return false; 
}

void KittingArm::clear_part_task(ariac_group1::PartTask& part_task, int& priority)
{
  m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks--; 
  ROS_INFO("Part left in shipment %s: %d", part_task.shipment_type.c_str(),
                                           m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks); 

  auto shipment_state = this->check_shipment_state(part_task); 
  this->process_shipment_state(shipment_state, part_task, priority); 
  if (shipment_state == ShipmentState::NOT_READY) {
    m_part_task_queue.pop_back();
  }
}

ShipmentState KittingArm::check_shipment_state(ariac_group1::PartTask& part_task)
{
  // Check if shipment is ready to submit
  if (m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks == 0) {
    ariac_group1::CheckQualitySensor srv; 
    srv.request.agv_id = part_task.agv_id; 
    if (m_check_quality_sensor_client.call(srv)) {

      if (srv.response.faulty_parts.empty()) {
        ROS_INFO("No faulty part in shipment %s", part_task.shipment_type.c_str()); 
        return ShipmentState::READY; 
      }
      else { 
        ROS_INFO("Has faulty in shipment %s", part_task.shipment_type.c_str()); 
        //push faulty task back to queue
        nist_gear::Model faulty_part = srv.response.faulty_parts[0]; 
        part_task.part.type = faulty_part.type;  
        part_task.part.pose = faulty_part.pose;  
        return ShipmentState::HAS_FAULTY; 
      }
    } 
    else {
      ROS_INFO("Sensor blackout, postpone shipment %s", part_task.shipment_type.c_str()); 
      return ShipmentState::POSTPONE; 
    }
  }
  else {
    return ShipmentState::NOT_READY; 
  }
}

void KittingArm::process_shipment_state(ShipmentState shipment_state, ariac_group1::PartTask& part_task, int& priority)
{
  if (shipment_state == ShipmentState::READY) {
    ros::Duration(1).sleep();

    // this->submit_shipment(part_task.agv_id, part_task.shipment_type, part_task.station_id); 
    m_agvs_dict[part_task.agv_id]->submit_shipment(part_task.shipment_type, part_task.station_id); 
    m_shipments.shipments_record[part_task.shipment_type]->state = ShipmentState::FINISH; 
    m_part_task_queue.pop_back(); 

  }
  else if (shipment_state == ShipmentState::HAS_FAULTY) {

    goToPresetLocation(part_task.agv_id);
    nist_gear::Model faulty_part; 
    faulty_part.type = part_task.part.type; 
    faulty_part.pose = part_task.part.pose; 
    bool discard_success = this->discard_faulty(faulty_part, part_task.agv_id); 
    auto pose_in_tray_frame = Utility::motioncontrol::transformToTrayFrame(faulty_part.pose, part_task.agv_id); 
    part_task.part.pose = pose_in_tray_frame; 
    if (discard_success) {
      // ready to replace the discard part
      m_shipments.shipments_record[part_task.shipment_type]->unfinished_part_tasks++; 
    }
  }
  else if (shipment_state == ShipmentState::POSTPONE) {

    priority += Constants::PriorityWeight::Penalty::SHIPMENT_POSTPONE; 

  }
}

// void KittingArm::submit_shipment(const std::string& agv_id, 
//                                  const std::string& shipment_type,  
//                                  const std::string& station_id)
// {
//   ROS_INFO("%s", shipment_type.c_str()); 
//   ROS_INFO("%s", station_id.c_str()); 
//
//   auto service_name = "/ariac/" + agv_id + "/submit_shipment"; 
//   auto client = m_nh.serviceClient<AGVToAssem>(service_name); 
//
//   // check if the client exists
//   if (!client.exists()) {
//     ROS_INFO("Waiting for the competition to be ready...");
//     client.waitForExistence();
//     ROS_INFO("Competition is now ready.");
//   }
//
//   AGVToAssem srv; 
//   srv.request.assembly_station_name = station_id;  
//   srv.request.shipment_type = shipment_type; 
//   // call the service to allow AGV to submit kitting shipment
//   if (client.call(srv)) {
//     ROS_INFO("Calling service %s", service_name.c_str()); 
//     ROS_INFO("%s", srv.response.message.c_str()); 
//   }
//   else{
//     ROS_ERROR("Failed to call %s", service_name.c_str()); 
//   }
//
// }
