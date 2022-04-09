#include "sensor_manager.h"

SensorManager::SensorManager(ros::NodeHandle* nodehandle):
  m_nh{*nodehandle}
{
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
