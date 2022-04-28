#include "sensors.h"

#include <tuple>
#include <cmath>
#include <string>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3

#include <nist_gear/Model.h>

Sensors::Sensors(ros::NodeHandle* nodehandle, const std::string &id): 
  m_nh{*nodehandle}, 
  m_id{id} {}

Sensors::~Sensors() {}

void Sensors::reset_blackout()
{
  m_blackout = true; 
}

bool Sensors::is_blackout()
{
  return m_blackout; 
}

LogicalCamera::LogicalCamera(ros::NodeHandle* nodehandle, const std::string& id): 
  Sensors(nodehandle, id)
{
    m_sensor_subscriber = m_nh.subscribe("/ariac/" + id, 10, &LogicalCamera::sensor_callback, this); 
    m_parts_publisher = m_nh.advertise<nist_gear::Model>("/database/parts", 10); 

    // Store the camera pose in world frame (camera pose is constant)
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Duration timeout(5.0);
    try {
      m_camera_frame = tfBuffer.lookupTransform("world", id + "_frame",
                                                   ros::Time(0), timeout);
    }
    catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      return; 
    }
}

void LogicalCamera::sensor_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
  m_blackout = false; 
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 

  // Clear and ready to recive new data
  m_parts_camera_frame.clear(); 

  // Store all parts in camera frame
  for (auto& model: msg->models){
    m_parts_camera_frame.emplace_back(std::make_unique<nist_gear::Model>(model)); 
  }
    
}

void LogicalCamera::camera_to_world()
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 

  double part_height = 0; 
  parts_world_frame.clear(); 
  for (auto &model: m_parts_camera_frame) {
    geometry_msgs::Pose model_pose = model->pose; 
    geometry_msgs::Pose transformed_model_pose; 

    // Calculate the part pose in world frame
    tf2::doTransform(model_pose, transformed_model_pose, m_camera_frame); 

    nist_gear::Model transformed_model; 
    transformed_model.type = model->type; 
    transformed_model.pose = transformed_model_pose; 
    parts_world_frame.emplace_back(std::make_unique<nist_gear::Model>(transformed_model)); 
    part_height = transformed_model.pose.position.z; 
    if (m_platform_height == -1) {
      m_platform_height = part_height; 
    }
  }
  
}


void LogicalCamera::update_parts(PartsDB& parts_database)
{
  // ROS_INFO("Sensor: %s", m_id.c_str()); 
  this->camera_to_world(); 
  for (auto &part: parts_world_frame){
    // erase null pointer in database
    // parts_database[part->type].erase(std::remove(parts_database[part->type].begin(),
    //                                              parts_database[part->type].end(),
    //                                              nullptr)); 
    //
    // m_parts_publisher.publish(*part); 
    ariac_group1::PartInfo part_info;  
    part_info.part = *part; 

    double roll, pitch, yaw;
    std::tie(roll, pitch, yaw) = Utility::quat_to_rpy(part->pose.orientation); 
    std::vector<double> rpy {roll, pitch, yaw}; 
    for (auto& angle: rpy) {
      if (angle < 0.02) {
        angle = 0; 
      }
    }
    part_info.roll = roll; 
    part_info.pitch = pitch; 
    part_info.yaw = yaw; 

    auto rectified_orientation = Utility::motioncontrol::quaternionFromEuler(roll, pitch, yaw);
    part_info.part.pose.orientation.x = rectified_orientation.getX();
    part_info.part.pose.orientation.y = rectified_orientation.getY();
    part_info.part.pose.orientation.z = rectified_orientation.getZ();
    part_info.part.pose.orientation.w = rectified_orientation.getW();
    

    part_info.faulty = false; 
    part_info.camera_id = m_id; 

    // double epsilon = 0.1; 
    // // only pump need to be flipped
    // auto found = part->type.find("pump");  
    // if ((abs(roll - M_PI) < epsilon) and (found != std::string::npos)) {
    //   ROS_INFO("flipped"); 
    //   part_info.flip = true; 
    // }
  
    if (parts_database.count(part->type)) {
      bool in_database = false; 
      for (auto& db_part_ptr: parts_database[part->type]) {
        if (Utility::is_same_part(part_info.part, db_part_ptr->part, 0.05)) {
          in_database = true; 
          break; 
        }
      }
      if (not in_database) {
          parts_database[part->type].emplace_back(std::make_unique<ariac_group1::PartInfo>(part_info)); 
      }
      // if (parts_database[part->type].size() <= 4) {
        // parts_database[part->type].emplace_back(std::make_unique<ariac_group1::PartInfo>(part_info)); 
      // }
    } else {
      parts_database[part->type].emplace_back(std::make_unique<ariac_group1::PartInfo>(part_info)); 
    }
  }
}

int LogicalCamera::find_parts(const std::string& product_type)
{
  this->camera_to_world(); 

  int count = 0; 
  for (auto &part: parts_world_frame){
    if (part->type == product_type) {
      double roll, pitch, yaw;
      // std::tie(roll, pitch, yaw) = Utility::quat_to_rpy(part->pose.orientation); 
      //
      // ROS_INFO("%s in /world frame: [%f,%f,%f] [%f,%f,%f]",
      //   part->type.c_str(), 
      //   part->pose.position.x,
      //   part->pose.position.y,
      //   part->pose.position.z,
      //   roll,
      //   pitch,
      //   yaw);
      //
      count++; 
    }
  }
  return count; 
}

DepthCamera::DepthCamera(ros::NodeHandle* nodehandle, const std::string& id):
  Sensors(nodehandle, id)
{
    m_sensor_subscriber = m_nh.subscribe("/ariac/" + id, 10, &DepthCamera::sensor_callback, this); 
}

void DepthCamera::sensor_callback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  m_blackout = false; 
   //ROS_INFO_STREAM_THROTTLE(10, m_id);
   //ROS_INFO_THROTTLE(1, "Callback triggered for Topic %s", m_id.c_str());
  return; 
}

ProximitySensor::ProximitySensor(ros::NodeHandle* nodehandle, const std::string& id): 
  Sensors(nodehandle, id)
{
    m_sensor_subscriber = m_nh.subscribe("/ariac/" + id, 10, &ProximitySensor::sensor_callback, this); 
}

void ProximitySensor::sensor_callback(const sensor_msgs::Range::ConstPtr& msg)
{
  m_blackout = false; 
  ROS_INFO_THROTTLE(1, "Callback triggered for Topic %s", m_id.c_str());
  if ((msg->max_range - msg->range) > 0.01) {
    // If there is an object in proximity.
    ROS_INFO_THROTTLE(1, "%s: sensed object", m_id.c_str());
  }
}

LaserProfiler::LaserProfiler(ros::NodeHandle* nodehandle, const std::string& id): 
  Sensors(nodehandle, id)
{
    m_sensor_subscriber = m_nh.subscribe("/ariac/" + id, 10, &LaserProfiler::sensor_callback, this); 
}

void LaserProfiler::sensor_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  m_blackout = false; 
  // size_t number_of_valid_ranges = std::count_if(
  //   msg->ranges.begin(), msg->ranges.end(), [](const float f)
  //   {
  //     return std::isfinite(f);
  //   });
  // if (number_of_valid_ranges > 0)
  // {
  //   ROS_INFO_THROTTLE(1, "%s: sensed object", m_id.c_str());
  // }
  ROS_INFO_THROTTLE(1, "Callback triggered for Topic %s", m_id.c_str());
}

BreakBeam::BreakBeam(ros::NodeHandle* nodehandle, const std::string& id): 
  Sensors(nodehandle, id)
{
    m_sensor_subscriber = m_nh.subscribe("/ariac/" + id, 10, &BreakBeam::sensor_callback, this); 
    m_sensor_change_subscriber = m_nh.subscribe("/ariac/" + id + "_change", 10, &BreakBeam::sensor_change_callback, this); 
}

void BreakBeam::sensor_callback(const nist_gear::Proximity::ConstPtr& msg)
{
  m_blackout = false; 
  // ROS_INFO_THROTTLE(1, "Callback triggered for Topic %s", m_id.c_str());
  // if (msg->object_detected) {  // If there is an object in proximity.
  //   ROS_INFO("%s triggered.", m_id.c_str());
  // }
}

void BreakBeam::sensor_change_callback(const nist_gear::Proximity::ConstPtr& msg)
{
  m_blackout = false; 
  // ROS_INFO_THROTTLE(1, "Callback triggered for Topic %s", m_id.c_str());
  if (msg->object_detected) {  // If there is an object in proximity.
    ROS_INFO("%s triggered.", (m_id + "_change").c_str());
    m_triggered = true;
  }
}

bool BreakBeam::is_triggered()
{
    return m_triggered; 
}

void BreakBeam::reset_triggered()
{
    m_triggered = false; 
}


