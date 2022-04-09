#include "sensor_manager.h"

SensorManager::SensorManager(ros::NodeHandle* nodehandle):
  m_nh{*nodehandle}
{
  m_get_parts_service = m_nh.advertiseService("/sensor_manager/get_parts", &SensorManager::get_parts, this); 

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
  for (auto& camera_id: m_logical_cameras) {
    m_logical_cameras_dict[camera_id]->update_parts(m_parts_database);  
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

bool SensorManager::get_parts(ariac_group1::GetParts::Request &req, 
                              ariac_group1::GetParts::Response &res) 
{
  for (auto& part_info_ptr: m_parts_database[req.type]) {
    res.parts_info.push_back(*part_info_ptr); 
    Utility::print_part_pose(part_info_ptr->part); 
  }

  return true; 
}


