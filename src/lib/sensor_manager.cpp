#include "sensor_manager.h"

#include <tuple>

SensorManager::SensorManager(ros::NodeHandle* nodehandle):
  m_nh{*nodehandle}
{
  m_get_parts_service = m_nh.advertiseService("/sensor_manager/get_parts", &SensorManager::get_parts, this); 
  m_is_faulty_service = m_nh.advertiseService("/sensor_manager/is_faulty", &SensorManager::is_faulty, this); 
  m_is_shipment_ready_service = m_nh.advertiseService("/sensor_manager/is_shipment_ready", &SensorManager::is_shipment_ready, this); 
  m_parts_in_camera_service = m_nh.advertiseService("/sensor_manager/parts_in_camera", &SensorManager::parts_in_camera, this); 
  m_is_part_picked_service = m_nh.advertiseService("/sensor_manager/is_part_picked", &SensorManager::is_part_picked, this); 

  // All Logical cameras in the environment
  for (auto& camera_id: m_logical_cameras) {
    std::string prefix = "logical_camera_"; 
    m_logical_cameras_dict[camera_id] = std::make_unique<LogicalCamera>(nodehandle, prefix += camera_id); 
  }

  // All quality sensors in the environment
  for (auto& camera_id: m_quality_sensors) {
    m_quality_sensors_dict[camera_id] = std::make_unique<LogicalCamera>(nodehandle, camera_id); 
  }

}

void SensorManager::update_parts()
{
  const std::lock_guard<std::mutex> lock(*m_mutex_ptr); 
  m_parts_database.clear(); 

  for (auto& camera_id: m_logical_cameras) {
    m_logical_cameras_dict[camera_id]->update_parts(m_parts_database);  
  }
  
  for (auto& camera_id: m_quality_sensors) {
    m_quality_sensors_dict[camera_id]->update_parts(m_parts_database);  
  }
}

void SensorManager::show_database()
{
  for (auto& data: m_parts_database) {
    auto key = data.first; 
    ROS_INFO("Key: %s", key.c_str()); 
    for (auto& part_info_ptr: data.second) {
      Utility::print_part_pose(part_info_ptr->part); 
    }
  }
  ROS_INFO("============="); 
}

void SensorManager::check_blackout() 
{
  static int test_count = 0; 

  for (auto& camera_id: m_logical_cameras) {
    if (not m_logical_cameras_dict[camera_id]->is_blackout()) {
      // If one of the sensors report false, then no blackout
      // set test_blackout for next time
      m_logical_cameras_dict[camera_id]->test_blackout(); 
      test_count = 0; 
      return; 
    }
  }

  if (test_count < 5) {

    test_count++; 

  } 
  else {

    ROS_INFO("Sensors blackout"); 
    test_count = 0; 

  }

}

bool SensorManager::get_parts(ariac_group1::GetParts::Request &req, 
                              ariac_group1::GetParts::Response &res) 
{
  for (auto& part_info_ptr: m_parts_database[req.type]) {
    res.parts_info.push_back(*part_info_ptr); 
    Utility::print_part_pose(part_info_ptr->part); 
  }

  return true; 
}

bool SensorManager::is_faulty(ariac_group1::IsFaulty::Request &req, 
                              ariac_group1::IsFaulty::Response &res) 
{
  for (auto& part_info_ptr: m_parts_database["model"]) {
    Utility::print_part_pose(part_info_ptr->part); 
    if (Utility::is_same_part(part_info_ptr->part, req.part, 0.2)) {
      res.faulty = true; 
      return true; 
    }
  }

  res.faulty = true; 

  return true; 
}

bool SensorManager::is_shipment_ready(ariac_group1::IsShipmentReady::Request &req, 
                                      ariac_group1::IsShipmentReady::Response &res) 
{
  auto agv_id = req.shipment.agv_id; 
  std::string camera_id; 
  if (agv_id == "agv1") {
    camera_id = "ks1"; 
  }
  if (agv_id == "agv2") {
    camera_id = "ks2"; 
  }
  if (agv_id == "agv3") {
    camera_id = "ks3"; 
  }
  if (agv_id == "agv4") {
    camera_id = "ks4"; 
  }

  bool ready = true; 
  for (auto& product: req.shipment.products) {
    nist_gear::Model order_part; 
    order_part.type = product.type; 
    order_part.pose = product.pose; 

    bool same_part = false; 
    for (auto& part_info_ptr: m_parts_database[product.type]) {
      if (Utility::is_same_part(order_part, part_info_ptr->part, 0.2)) {
        same_part = true; 
        break; 
      }
    }
    if (not same_part) {
      ready = false; 
      break; 
    }
  }
  res.ready = ready; 
  return true; 
}

bool SensorManager::parts_in_camera(ariac_group1::PartsInCamera::Request &req, 
                                    ariac_group1::PartsInCamera::Response &res) 
{
  ros::spinOnce(); 
  std::string id = this->convert_id_to_internal(req.camera_id); 
  if (m_logical_cameras_dict.count(id)) {
    res.parts_amount = m_logical_cameras_dict[id]->parts_world_frame.size(); 
    return true; 
  }

  if (m_quality_sensors_dict.count(id)) {
    res.parts_amount = m_quality_sensors_dict[id]->parts_world_frame.size(); 
    return true; 
  }

  return false; 
}

std::string SensorManager::convert_id_to_internal(std::string global_id)
{
  std::string internal_id; 
  if (global_id.compare("logical_camera_bins0") == 0) {
    internal_id = "bins0"; 
  }
  else if (global_id.compare("logical_camera_bins1") == 0) {
    internal_id = "bins1"; 
  }
  else if (global_id.compare("logical_camera_ks1") == 0) {
    internal_id = "ks1"; 
  }
  else if (global_id.compare("logical_camera_ks2") == 0) {
    internal_id = "ks2"; 
  }
  else if (global_id.compare("logical_camera_ks3") == 0) {
    internal_id = "ks3"; 
  }
  else if (global_id.compare("logical_camera_ks4") == 0) {
    internal_id = "ks4"; 
  }

  if (global_id.find("quality_control_sensor") != std::string::npos) {
    internal_id = global_id; 
  }

  return internal_id; 
}

bool SensorManager::is_part_picked(ariac_group1::IsPartPicked::Request &req,
                                   ariac_group1::IsPartPicked::Response &res) 
{
  std::string id = this->convert_id_to_internal(req.camera_id); 
  double platform_height = -1; 
  if (id.find("ks") != std::string::npos) {
    platform_height = 0.78;  
  }
  else if (id.find("bins") != std::string::npos) {
    platform_height = 0.75;  
  }
  
  ros::spinOnce(); 
  res.picked = false; 
  if (m_logical_cameras_dict.count(id)) {
    for (auto &part: m_logical_cameras_dict[id]->parts_world_frame) {
      if (part->type != req.part_type) {
        continue; 
      }
      double picked_margin = 0.1; 
      if ((part->pose.position.z - platform_height) > picked_margin) {
        res.picked = true; 
      }
    }
    return true; 
  }

  return false; 
}


