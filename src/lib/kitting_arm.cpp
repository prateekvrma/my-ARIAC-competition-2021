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

#include <nist_gear/AGVToAssemblyStation.h>
#include <ariac_group1/IsFaulty.h>

using AGVToAssem = nist_gear::AGVToAssemblyStation; 

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


  m_get_parts_client = 
      m_nh.serviceClient<ariac_group1::GetParts>("/sensor_manager/get_parts"); 
  m_get_parts_client.waitForExistence();

  m_is_faulty_client = 
      m_nh.serviceClient<ariac_group1::IsFaulty>("/sensor_manager/is_faulty"); 
  m_get_parts_client.waitForExistence();


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
  location_agv4.joints_position.at(0) = -4.1; 
  location_agv4.name = "agv4"; 

  location_bins0.joints_position = home_face_bins.joints_position; 
  // linear actuator at y axis 
  location_bins0.joints_position.at(0) = 3; 
  location_bins0.name = "bins0"; 

  location_bins1.joints_position = home_face_bins.joints_position; 
  // linear actuator at y axis 
  location_bins1.joints_position.at(0) = -3; 
  location_bins1.name = "bins1"; 

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
  m_part_task_queue.emplace_back(std::make_tuple(msg->priority, std::make_unique<ariac_group1::PartTask>(*msg))); 
  if (not m_shipments_total_parts.count(msg->shipment_type)) {
    m_shipments_total_parts[msg->shipment_type] = msg->total_parts; 
  }
  this->print_shipments_total_parts(); 
  Utility::print_part_pose(msg->part); 
}

void KittingArm::print_shipments_total_parts() {
  for (auto& part_count: m_shipments_total_parts) {
    ROS_INFO("%s: %d", part_count.first.c_str(), part_count.second); 
  }

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

bool KittingArm::pickPart(std::string part_type, 
                          const geometry_msgs::Pose& part_init_pose) 
{
    m_arm_group.setMaxVelocityScalingFactor(1.0);


    moveBaseTo(part_init_pose.position.y);

    // // move the arm above the part to grasp
    // // gripper stays at the current z
    // // only modify its x and y based on the part to grasp
    // // In this case we do not need to use preset locations
    // // everything is done dynamically
    // arm_ee_link_pose.position.x = part_init_pose.position.x;
    // arm_ee_link_pose.position.y = part_init_pose.position.y;
    // arm_ee_link_pose.position.z = arm_ee_link_pose.position.z;
    // // move the arm
    // arm_group_.setPoseTarget(arm_ee_link_pose);
    // arm_group_.move();

    // Make sure the wrist is facing down
    // otherwise it will have a hard time attaching a part
    geometry_msgs::Pose arm_ee_link_pose = m_arm_group.getCurrentPose().pose;
    auto flat_orientation = motioncontrol::quaternionFromEuler(0, 1.57, 0);
    arm_ee_link_pose.orientation.x = flat_orientation.getX();
    arm_ee_link_pose.orientation.y = flat_orientation.getY();
    arm_ee_link_pose.orientation.z = flat_orientation.getZ();
    arm_ee_link_pose.orientation.w = flat_orientation.getW();
    
    // post-grasp pose 3
    // store the pose of the arm before it goes down to pick the part
    // we will bring the arm back to this pose after picking up the part
    auto postgrasp_pose3 = part_init_pose;
    postgrasp_pose3.orientation = arm_ee_link_pose.orientation;
    postgrasp_pose3.position.z = arm_ee_link_pose.position.z;

    // preset z depending on the part type
    // some parts are bigger than others
    // TODO: Add new z_pos values for the regulator and the battery
    double z_pos{};
    if (part_type.find("pump") != std::string::npos) {
        z_pos = 0.859;
    }
    if (part_type.find("sensor") != std::string::npos) {
        z_pos = 0.81;
    }
    if (part_type.find("battery") != std::string::npos) {
        z_pos = 0.81;
    }
    if (part_type.find("regulator") != std::string::npos) {
        z_pos = 0.81;
    }

    // flat_orientation = motioncontrol::quaternionFromEuler(0, 1.57, 0);
    // arm_ee_link_pose = arm_group_.getCurrentPose().pose;
    // arm_ee_link_pose.orientation.x = flat_orientation.getX();
    // arm_ee_link_pose.orientation.y = flat_orientation.getY();
    // arm_ee_link_pose.orientation.z = flat_orientation.getZ();
    // arm_ee_link_pose.orientation.w = flat_orientation.getW();

    
    // set of waypoints the arm will go through
    std::vector<geometry_msgs::Pose> waypoints;
    // pre-grasp pose: somewhere above the part
    auto pregrasp_pose = part_init_pose;
    pregrasp_pose.orientation = arm_ee_link_pose.orientation;
    pregrasp_pose.position.z = z_pos + 0.06;

    // grasp pose: right above the part
    auto grasp_pose = part_init_pose;
    grasp_pose.orientation = arm_ee_link_pose.orientation;
    grasp_pose.position.z = z_pos + 0.03;

    waypoints.push_back(pregrasp_pose);
    waypoints.push_back(grasp_pose);

    // activate gripper
    // sometimes it does not activate right away
    // so we are doing this in a loop
    while (!m_gripper_state.enabled) {
        activateGripper();
    }

    // move the arm to the pregrasp pose
    m_arm_group.setPoseTarget(pregrasp_pose);
    m_arm_group.move();

    
    /* Cartesian motions are frequently needed to be slower for actions such as approach
    and retreat grasp motions. Here we demonstrate how to reduce the speed and the acceleration
    of the robot arm via a scaling factor of the maxiumum speed of each joint.
    */
    m_arm_group.setMaxVelocityScalingFactor(0.05);
    m_arm_group.setMaxAccelerationScalingFactor(0.05);
    // plan the cartesian motion and execute it
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = m_arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    m_arm_group.execute(plan);

    ros::Duration(sleep(2.0));

    // move the arm 1 mm down until the part is attached
    while (!m_gripper_state.attached) {
        grasp_pose.position.z -= 0.001;
        m_arm_group.setPoseTarget(grasp_pose);
        m_arm_group.move();
        ros::Duration(sleep(0.5));
    }
    
    m_arm_group.setMaxVelocityScalingFactor(1.0);
    m_arm_group.setMaxAccelerationScalingFactor(1.0);
    ROS_INFO_STREAM("[Gripper] = object attached");
    ros::Duration(sleep(2.0));
    m_arm_group.setPoseTarget(pregrasp_pose);
    m_arm_group.setPoseTarget(postgrasp_pose3);
    m_arm_group.move();

    return true;
    
}

geometry_msgs::Pose KittingArm::placePart(geometry_msgs::Pose part_init_pose, 
                                          geometry_msgs::Pose part_pose_in_frame, 
                                          std::string agv)
{
    goToPresetLocation(agv);
    
    // get the target pose of the part in the world frame
    auto target_pose_in_world = motioncontrol::transformToWorldFrame(
         part_pose_in_frame,
         agv);

    ROS_INFO("Part goal:"); 
    Utility::print_pose(target_pose_in_world); 

   

    geometry_msgs::Pose arm_ee_link_pose = m_arm_group.getCurrentPose().pose;
    auto flat_orientation = motioncontrol::quaternionFromEuler(0, 1.57, 0);
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
    m_arm_group.setMaxVelocityScalingFactor(1.0);
    m_arm_group.setPoseTarget(arm_ee_link_pose);
    m_arm_group.move();

  
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

    // orientation of the gripper when placing the part in the tray
    target_pose_in_world.orientation.x = q_rslt.x();
    target_pose_in_world.orientation.y = q_rslt.y();
    target_pose_in_world.orientation.z = q_rslt.z();
    target_pose_in_world.orientation.w = q_rslt.w();
    target_pose_in_world.position.z += 0.2;

    m_arm_group.setMaxVelocityScalingFactor(0.1);
    m_arm_group.setPoseTarget(target_pose_in_world);
    m_arm_group.move();
    ros::Duration(5.0).sleep();
    deactivateGripper();
    //
    m_arm_group.setMaxVelocityScalingFactor(1.0);

    return target_pose_in_world;
}
void KittingArm::discard_faulty(std::string part_type, geometry_msgs::Pose part_init_pose)
{
      pickPart(part_type, part_init_pose);
      goToPresetLocation("home_face_bins");
      ros::Duration(5.0).sleep();
      deactivateGripper();
}

bool KittingArm::movePart(const ariac_group1::PartInfo& part_init_info, const ariac_group1::PartTask& part_task) {
    auto init_pose_in_world = part_init_info.part.pose;
    auto part_type = part_init_info.part.type; 

    if (pickPart(part_type, init_pose_in_world)) {
        auto target_pose = placePart(init_pose_in_world, part_task.part.pose, part_task.agv_id);

        ariac_group1::IsFaulty srv; 
        nist_gear::Model faulty_part; 
        faulty_part.type = part_type; 
        faulty_part.pose = target_pose; 
        srv.request.part = faulty_part; 
        if (m_is_faulty_client.call(srv)) {
          ROS_INFO("Check faulty"); 
          Utility::print_part_pose(faulty_part); 
          if (srv.response.faulty) {
            ROS_INFO("=====================Faulty========================="); 
            return false; 
          }
        }

    }
    return true; 
}

bool KittingArm::get_order()
{
  ros::Rate wait_rate(20); 
  // check if there are tasks and make sure ROS is running 
  while (m_part_task_queue.empty() && ros::ok()) {
    ROS_INFO_THROTTLE(3, "Waiting for part task.");
    
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

  ROS_INFO("Shipment: %s", part_task.shipment_type.c_str()); 
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
      idx++; 
      part_init_info = parts_info[idx]; 
      if (idx == parts_info.size()) {
        if (priority > 0) {
          ROS_INFO("High priority gets part from agv"); 
          break; 
        }
        ROS_INFO("No enough part for %s", part_task.part.type.c_str()); 
        return; 
      }
    }

    ROS_INFO("Found %s upder %s", part_task.part.type.c_str(), part_init_info.camera_id.c_str()); 
    ROS_INFO("Part location: "); 
    Utility::print_part_pose(part_init_info.part); 
     
    bool success = this->movePart(part_init_info, part_task); 
    if (success) {
      ROS_INFO("Success moving part"); 
      m_shipments_total_parts[part_task.shipment_type]--; 
      ROS_INFO("Part left: %d",m_shipments_total_parts[part_task.shipment_type]); 
      if (m_shipments_total_parts[part_task.shipment_type] == 0) {
        this->submit_shipment(part_task.agv_id, part_task.shipment_type, part_task.station_id); 
      }
      m_part_task_queue.pop_back(); 
      return; 
    }
    ROS_INFO("movement false"); 

  }
  else {
    ROS_INFO("Not Found %s, back to task queue", part_task.part.type.c_str()); 
    return; 
  }

}

void KittingArm::submit_shipment(const std::string& agv_id, 
                                 const std::string& shipment_type,  
                                 const std::string& station_id)
{
  ROS_INFO("%s", shipment_type.c_str()); 
  ROS_INFO("%s", station_id.c_str()); 

  auto service_name = "/ariac/" + agv_id + "/submit_shipment"; 
  auto client = m_nh.serviceClient<AGVToAssem>(service_name); 

  // check if the client exists
  if (!client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }

  AGVToAssem srv; 
  srv.request.assembly_station_name = station_id;  
  srv.request.shipment_type = shipment_type; 
  // call the service to allow AGV to submit kitting shipment
  if (client.call(srv)) {
    ROS_INFO("Calling service %s", service_name.c_str()); 
    ROS_INFO("%s", srv.response.message.c_str()); 
  }
  else{
    ROS_ERROR("Failed to call %s", service_name.c_str()); 
  }

}
